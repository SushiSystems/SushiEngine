/**************************************************************************/
/* test_physics_assembly.cpp                                              */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// The PhysicsAssembly asset and the ragdoll built on it, both host-only: no runtime,
// no device, no solver. What is checked here is the translation — an authored assembly
// into a blob and back, a blob plus a root pose into bodies and joints, and a skeleton
// into a simulable rig whose bind offsets recover the pose they were measured from.
//
// The ragdoll half is arranged around one property worth stating plainly: at the bind
// pose, resolving the targets must reproduce the skeleton's own bind model transforms
// exactly. That is the round trip the whole binding exists for, and it is the one thing
// that cannot be got right by inspection — a capsule's segment runs along its own local
// +Y, so a part's orientation is not the joint's, and every sign in the offset has to
// be right for the two to compose back.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/skeleton_blob.hpp>
#include <SushiEngine/simulation/physics_assembly.hpp>
#include <SushiEngine/simulation/ragdoll.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief A box part at @p position with @p half_extents. */
    AssemblyPart box_part(const Vector3& position, const Vector3& half_extents,
                          std::uint32_t group = 0)
    {
        AssemblyPart part;
        part.collider.shape = ColliderShape::Box;
        part.collider.half_extents = half_extents;
        part.local_position = position;
        part.group = group;
        return part;
    }

    /** @brief A two-part assembly on one self-excluding group, held by a hinge. */
    PhysicsAssembly two_part_assembly()
    {
        PhysicsAssembly assembly;
        assembly.parts.push_back(box_part(Vector3{0, 0, 0}, Vector3{1, Scalar(0.6), Scalar(0.5)}));
        assembly.parts.push_back(
            box_part(Vector3{Scalar(1.5), 0, 0}, Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.05)}));
        assembly.parts[0].inv_mass = 0;
        assembly.parts[1].inv_mass = Scalar(1) / Scalar(35);
        assembly.part_names.push_back("chassis");
        assembly.part_names.push_back("door");

        AssemblyJoint hinge;
        hinge.part_a = 0;
        hinge.part_b = 1;
        hinge.params.type = JointType::Hinge;
        hinge.params.anchor_a = Vector3{1, 0, 0};
        hinge.params.anchor_b = Vector3{Scalar(-0.5), 0, 0};
        hinge.params.axis_a = Vector3{0, 1, 0};
        hinge.params.axis_b = Vector3{0, 1, 0};
        hinge.params.twist_limit.enabled = true;
        hinge.params.twist_limit.upper = Scalar(1.18682);
        hinge.params.break_force = Scalar(12000);
        assembly.joints.push_back(hinge);

        assembly.group_masks.assign(1, assembly_group_excluding_self(0));
        return assembly;
    }

    /**
     * @brief A nine-joint humanoid-shaped skeleton.
     *
     * Shaped to exercise the cases rather than to look like a person: `chest` has two
     * children so the mean-of-children bone is used, and `head` and `wrist` are leaves
     * so the no-part path is taken.
     */
    Animation::SkeletonDescription humanoid_desc()
    {
        const auto joint = [](const char* name, int parent, Scalar x, Scalar y, Scalar z)
        {
            Animation::JointDescription desc;
            desc.name = name;
            desc.parent = parent;
            desc.bind_translation =
                Animation::Vector3f{float(x), float(y), float(z)};
            return desc;
        };

        Animation::SkeletonDescription desc;
        desc.joints.push_back(joint("root", -1, 0, 0, 0));
        desc.joints.push_back(joint("hip", 0, 0, Scalar(0.9), 0));
        desc.joints.push_back(joint("spine", 1, 0, Scalar(0.2), 0));
        desc.joints.push_back(joint("chest", 2, 0, Scalar(0.25), 0));
        desc.joints.push_back(joint("neck", 3, 0, Scalar(0.2), 0));
        desc.joints.push_back(joint("head", 4, 0, Scalar(0.12), 0));
        desc.joints.push_back(joint("shoulder", 3, Scalar(0.18), Scalar(0.05), 0));
        desc.joints.push_back(joint("elbow", 6, Scalar(0.28), 0, 0));
        desc.joints.push_back(joint("wrist", 7, Scalar(0.25), 0, 0));
        return desc;
    }

    /** @brief Holds a skeleton blob alive for as long as the view over it. */
    struct Humanoid
    {
        std::vector<std::byte> blob;
        Animation::SkeletonView view;

        Humanoid()
        {
            const bool built = Animation::build_skeleton_blob(humanoid_desc(), blob);
            if (built)
                view = Animation::load_skeleton_blob(blob.data(), blob.size());
        }
    };

    /** @brief The skeleton's bind pose in object space, composed independently. */
    void bind_pose(const Animation::SkeletonView& skeleton, std::vector<Vector3>& positions,
                   std::vector<Quaternion>& rotations)
    {
        positions.assign(skeleton.joint_count, Vector3{});
        rotations.assign(skeleton.joint_count, Quaternion{});
        for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
        {
            const Animation::Vector3f& t = skeleton.bind_translations[i];
            const Animation::Quaternionf& r = skeleton.bind_rotations[i];
            const Vector3 local_t{Scalar(t.x), Scalar(t.y), Scalar(t.z)};
            const Quaternion local_r{Scalar(r.x), Scalar(r.y), Scalar(r.z), Scalar(r.w)};
            const std::uint16_t parent = skeleton.parents[i];
            if (parent == Animation::NO_PARENT)
            {
                positions[i] = local_t;
                rotations[i] = local_r;
                continue;
            }
            positions[i] = positions[parent] + rotate(rotations[parent], local_t);
            rotations[i] = mul(rotations[parent], local_r);
        }
    }

    /**
     * @brief The narrowest `IRigidBodyService` that can answer a pose question.
     *
     * A stub rather than a real simulation, because what `resolve_ragdoll_targets` is
     * being held to is the transform algebra and not the solve: given known part poses,
     * the targets must come back in object space. Standing up a device to supply poses a
     * test already knows would be measuring the runtime.
     */
    class StubBodies final : public IRigidBodyService
    {
        public:
            std::unordered_map<EntityId, SolvedPose> poses;

            void set_rigid_bodies(const std::vector<RigidBodyDescription>&, std::size_t,
                                  Scalar) override
            {
            }
            void update_rigid_body_params(EntityId, Scalar, const Vector3&, Scalar) override {}
            void set_rigid_pose(EntityId id, const Vector3& position,
                                const Quaternion& orientation) override
            {
                poses[id] = SolvedPose{position, orientation};
            }
            bool rigid_pose(EntityId id, SolvedPose& out) const override
            {
                const auto it = poses.find(id);
                if (it == poses.end())
                    return false;
                out = it->second;
                return true;
            }
            bool rigid_debug_state(EntityId, RigidDebugState&) const override { return false; }
    };

    /** @brief Entity ids `1..count`, the shape a caller hands instancing. */
    std::vector<EntityId> part_entities(std::size_t count)
    {
        std::vector<EntityId> ids;
        for (std::size_t i = 0; i < count; ++i)
            ids.push_back(EntityId(i + 1));
        return ids;
    }

    double distance(const Vector3& a, const Vector3& b)
    {
        return double(length(a - b));
    }

    /** @brief The translation a Matrix4 carries (column-major). */
    Vector3 translation_of(const Matrix4& matrix)
    {
        return Vector3{Scalar(matrix.m[12]), Scalar(matrix.m[13]), Scalar(matrix.m[14])};
    }
}

// -- The blob --------------------------------------------------------------

TEST(Unit_PhysicsAssembly, TheBlobRoundTripsEveryPartAndJoint)
{
    const PhysicsAssembly authored = two_part_assembly();

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_assembly_blob(authored, blob));
    ASSERT_TRUE(validate_assembly_blob(blob.data(), blob.size()));

    const PhysicsAssemblyView view = load_assembly_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid());
    ASSERT_EQ(view.part_count, 2u);
    ASSERT_EQ(view.joint_count, 1u);
    ASSERT_EQ(view.group_count, 1u);

    EXPECT_EQ(int(view.parts[1].collider.shape), int(ColliderShape::Box));
    EXPECT_NEAR(double(view.parts[1].local_position.x), 1.5, 1e-12);
    EXPECT_NEAR(double(view.parts[1].inv_mass), 1.0 / 35.0, 1e-12);
    EXPECT_EQ(double(view.parts[0].inv_mass), 0.0);

    EXPECT_EQ(view.joints[0].part_a, 0u);
    EXPECT_EQ(view.joints[0].part_b, 1u);
    EXPECT_EQ(int(view.joints[0].params.type), int(JointType::Hinge));
    EXPECT_TRUE(view.joints[0].params.twist_limit.enabled);
    EXPECT_NEAR(double(view.joints[0].params.twist_limit.upper), 1.18682, 1e-9);
    EXPECT_NEAR(double(view.joints[0].params.break_force), 12000.0, 1e-9);

    // The group's own bit is clear, which is the whole of "the parts of one assembly do
    // not push each other".
    EXPECT_EQ(view.group_masks[0] & 1u, 0u);
    EXPECT_EQ(assembly_group_mask(view, 0) & 1u, 0u);
    // A group past the end is permissive rather than silently blocking everything: a
    // filter that dropped every contact would be far harder to notice than one that
    // dropped none.
    EXPECT_EQ(assembly_group_mask(view, 7), 0xFFFFFFFFu);
}

TEST(Unit_PhysicsAssembly, ThePartNamesSurviveForTheEditorToShow)
{
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_assembly_blob(two_part_assembly(), blob));
    const PhysicsAssemblyView view = load_assembly_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid());

    EXPECT_STREQ(assembly_part_name(view, 0), "chassis");
    EXPECT_STREQ(assembly_part_name(view, 1), "door");
    // Out of range answers an empty string rather than walking off the section.
    EXPECT_STREQ(assembly_part_name(view, 9), "");
}

TEST(Unit_PhysicsAssembly, TheWriterRefusesWhatTheLoaderWouldRefuse)
{
    std::vector<std::byte> blob;

    // No parts at all: an assembly of nothing has nothing to instance.
    EXPECT_FALSE(build_assembly_blob(PhysicsAssembly{}, blob));
    EXPECT_TRUE(blob.empty());

    // A joint naming a part that does not exist. Unchecked, its symptom is a joint
    // silently projected against part 0.
    PhysicsAssembly dangling = two_part_assembly();
    dangling.joints[0].part_b = 7;
    EXPECT_FALSE(build_assembly_blob(dangling, blob));
    EXPECT_TRUE(blob.empty());

    // A joint from a part to itself is not degenerate, it is a constraint the solver
    // would colour against one body twice.
    PhysicsAssembly self = two_part_assembly();
    self.joints[0].part_b = self.joints[0].part_a;
    EXPECT_FALSE(build_assembly_blob(self, blob));
    EXPECT_TRUE(blob.empty());
}

TEST(Unit_PhysicsAssembly, TheLoaderRefusesATruncatedOrForeignBlob)
{
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_assembly_blob(two_part_assembly(), blob));

    EXPECT_FALSE(validate_assembly_blob(nullptr, blob.size()));
    EXPECT_FALSE(validate_assembly_blob(blob.data(), sizeof(AssemblyBlobHeader) - 1));
    // Truncated: the header's declared total exceeds what is actually there. This is the
    // case a header that lies about its length produces.
    EXPECT_FALSE(validate_assembly_blob(blob.data(), blob.size() - 1));
    EXPECT_FALSE(load_assembly_blob(blob.data(), blob.size() - 1).valid());

    std::vector<std::byte> wrong_magic = blob;
    wrong_magic[0] = std::byte{'X'};
    EXPECT_FALSE(validate_assembly_blob(wrong_magic.data(), wrong_magic.size()));

    std::vector<std::byte> wrong_version = blob;
    AssemblyBlobHeader header{};
    std::memcpy(&header, wrong_version.data(), sizeof(header));
    header.version = ASSEMBLY_BLOB_VERSION + 1;
    std::memcpy(wrong_version.data(), &header, sizeof(header));
    EXPECT_FALSE(validate_assembly_blob(wrong_version.data(), wrong_version.size()));
}

TEST(Unit_PhysicsAssembly, TheLoaderRefusesAHandEditedDanglingJoint)
{
    // The joints are validated on load as well as on write, because a blob may have come
    // from an older writer or been edited by hand — and this is what makes the loader's
    // guarantee strong enough for `instantiate_assembly` to index without checking.
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_assembly_blob(two_part_assembly(), blob));

    AssemblyBlobHeader header{};
    std::memcpy(&header, blob.data(), sizeof(header));
    AssemblyJoint joint{};
    std::memcpy(&joint, blob.data() + header.joints_offset, sizeof(joint));
    joint.part_b = 9;
    std::memcpy(blob.data() + header.joints_offset, &joint, sizeof(joint));

    EXPECT_FALSE(validate_assembly_blob(blob.data(), blob.size()));
}

// -- Instancing -----------------------------------------------------------

TEST(Unit_PhysicsAssembly, InstancingPlacesEveryPartRelativeToTheRoot)
{
    const PhysicsAssembly assembly = two_part_assembly();
    const std::vector<EntityId> ids = part_entities(assembly.parts.size());

    // A root that is both moved and turned, so a part placed by addition alone would
    // land in the wrong place and be visible immediately.
    const Vector3 root{Scalar(10), Scalar(2), Scalar(-3)};
    const Quaternion turn = quaternion_axis_angle(Vector3{0, 1, 0}, Scalar(3.14159265358979 / 2));

    const AssemblyInstantiation out =
        instantiate_assembly(to_view(assembly), ids.data(), ids.size(), root, turn);
    ASSERT_EQ(out.bodies.size(), 2u);

    EXPECT_EQ(out.bodies[0].id, ids[0]);
    EXPECT_LT(distance(out.bodies[0].position, root), 1e-12);

    // The door is 1.5 m along the assembly's local +x; a quarter turn about +y puts that
    // along world -z.
    const Vector3 expected = root + Vector3{0, 0, Scalar(-1.5)};
    EXPECT_LT(distance(out.bodies[1].position, expected), 1e-9);

    // And its orientation is the root's composed with its own, not either alone.
    const Vector3 door_x = rotate(out.bodies[1].orientation, Vector3{1, 0, 0});
    EXPECT_NEAR(double(door_x.z), -1.0, 1e-9);
}

TEST(Unit_PhysicsAssembly, InstancingLetsTheFilterMatrixOverrideTheAuthoredCollider)
{
    PhysicsAssembly assembly = two_part_assembly();
    // An authored collider that says the opposite of the matrix. The matrix must win,
    // or "part 0 and part 1 do not collide" would be answered in two places.
    assembly.parts[0].collider.filter.layer = 1u << 9;
    assembly.parts[0].collider.filter.collides_with = 0xFFFFFFFFu;

    const std::vector<EntityId> ids = part_entities(assembly.parts.size());
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    ASSERT_EQ(out.bodies.size(), 2u);

    EXPECT_EQ(out.bodies[0].collider.filter.layer, 1u);
    EXPECT_EQ(out.bodies[0].collider.filter.collides_with & 1u, 0u);
    EXPECT_EQ(out.bodies[1].collider.filter.layer, 1u);
    EXPECT_EQ(out.bodies[1].collider.filter.collides_with & 1u, 0u);
}

TEST(Unit_PhysicsAssembly, InstancingDerivesMassFromDensityAndOtherwiseKeepsWhatWasAuthored)
{
    PhysicsAssembly assembly = two_part_assembly();
    const std::vector<EntityId> ids = part_entities(assembly.parts.size());

    // Density zero: the authored inverse mass survives untouched, which is the same rule
    // an ordinary body follows — a derived mass nobody asked for looks derived and is
    // therefore worse than a hand-typed one.
    const AssemblyInstantiation authored = instantiate_assembly(
        to_view(assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    EXPECT_EQ(double(authored.bodies[0].inv_mass), 0.0);
    EXPECT_NEAR(double(authored.bodies[1].inv_mass), 1.0 / 35.0, 1e-12);

    // A density on the door: a 1 x 1 x 0.1 box at 500 kg/m^3 weighs 50 kg.
    assembly.parts[1].density = Scalar(500);
    const AssemblyInstantiation derived = instantiate_assembly(
        to_view(assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    EXPECT_NEAR(double(derived.bodies[1].inv_mass), 1.0 / 50.0, 1e-9);
    EXPECT_GT(double(derived.bodies[1].inv_inertia.x), 0.0);
    // The pinned chassis still has no density, so it is still pinned.
    EXPECT_EQ(double(derived.bodies[0].inv_mass), 0.0);
}

TEST(Unit_PhysicsAssembly, InstancingNamesTheEntitiesItsPartsBecame)
{
    const PhysicsAssembly assembly = two_part_assembly();
    const std::vector<EntityId> ids = part_entities(assembly.parts.size());
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});

    ASSERT_EQ(out.joints.size(), 1u);
    EXPECT_EQ(out.joints[0].body_a, ids[0]);
    EXPECT_EQ(out.joints[0].body_b, ids[1]);
    // And the parameters travelled whole, which is what sharing `JointParameters` with a
    // hand-built joint buys: nothing here copies a field, so nothing here can forget one.
    EXPECT_EQ(int(out.joints[0].params.type), int(JointType::Hinge));
    EXPECT_NEAR(double(out.joints[0].params.break_force), 12000.0, 1e-9);
    EXPECT_NEAR(double(out.joints[0].params.anchor_a.x), 1.0, 1e-12);
}

TEST(Unit_PhysicsAssembly, InstancingRefusesRatherThanHalfBuildAnAssembly)
{
    const PhysicsAssembly assembly = two_part_assembly();
    const std::vector<EntityId> ids = part_entities(1);

    // One entity for two parts. A partial instance is worse than none: its joints would
    // name entities that do not exist.
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    EXPECT_TRUE(out.bodies.empty());
    EXPECT_TRUE(out.joints.empty());

    const AssemblyInstantiation none = instantiate_assembly(
        PhysicsAssemblyView{}, ids.data(), ids.size(), Vector3{}, Quaternion{});
    EXPECT_TRUE(none.bodies.empty());
}

// -- The ragdoll ----------------------------------------------------------

TEST(Unit_PhysicsAssembly, ARagdollGivesAPartToEveryBoneAndNoneToALeaf)
{
    const Humanoid humanoid;
    ASSERT_EQ(humanoid.view.joint_count, 9u);

    const RagdollRig rig = build_ragdoll_rig(humanoid.view, RagdollProfile{});

    // root, hip, spine, chest, neck, shoulder, elbow have children; head and wrist do
    // not, and get no part — RagdollBlend's recompose carries a physics-driven parent
    // down to them, which is what a fingertip should do.
    ASSERT_EQ(rig.assembly.parts.size(), 7u);
    ASSERT_EQ(rig.bindings.size(), 7u);
    ASSERT_EQ(rig.part_of_joint.size(), 9u);

    // By name, never by authored index: the skeleton cook sorts and remaps joints
    // topologically, so the order a desc was written in is not the order a rig is built
    // against. A test that hard-codes indices is testing the cook's sort.
    const auto joint_of = [&humanoid](const char* name)
    {
        const int index = humanoid.view.find_joint(Animation::hash_name(name));
        return std::size_t(index < 0 ? 0 : index);
    };
    EXPECT_EQ(rig.part_of_joint[joint_of("head")], NO_RAGDOLL_PART) << "head is a leaf";
    EXPECT_EQ(rig.part_of_joint[joint_of("wrist")], NO_RAGDOLL_PART) << "wrist is a leaf";
    for (const char* name : {"root", "hip", "spine", "chest", "neck", "shoulder", "elbow"})
        EXPECT_NE(rig.part_of_joint[joint_of(name)], NO_RAGDOLL_PART) << name;

    // One joint per part except the root's, which has no ancestor to hang from.
    EXPECT_EQ(rig.assembly.joints.size(), 6u);

    // The part names came from the skeleton, so the assembly editor's parts list reads
    // like the rig rather than like an array.
    EXPECT_EQ(rig.assembly.part_names[rig.part_of_joint[joint_of("root")]], "root");
    EXPECT_EQ(rig.assembly.part_names[rig.part_of_joint[joint_of("chest")]], "chest");
}

TEST(Unit_PhysicsAssembly, EachCapsuleSpansItsOwnBone)
{
    const Humanoid humanoid;
    RagdollProfile profile;
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, profile);
    ASSERT_FALSE(rig.assembly.parts.empty());

    std::vector<Vector3> positions;
    std::vector<Quaternion> rotations;
    bind_pose(humanoid.view, positions, rotations);

    for (const RagdollBinding& binding : rig.bindings)
    {
        // The bone this part stands for: from its joint to the mean of its children.
        Vector3 sum{};
        std::uint32_t children = 0;
        for (std::uint32_t j = 0; j < humanoid.view.joint_count; ++j)
        {
            if (humanoid.view.parents[j] != binding.joint)
                continue;
            sum = sum + positions[j];
            ++children;
        }
        ASSERT_GT(children, 0u);
        const Vector3 bone =
            sum * (Scalar(1) / Scalar(children)) - positions[binding.joint];
        const double bone_length = double(length(bone));

        const AssemblyPart& part = rig.assembly.parts[binding.part];
        // A capsule's length is its segment plus a cap at each end.
        const double capsule_length =
            2.0 * double(part.collider.half_height) + 2.0 * double(part.collider.radius);
        EXPECT_NEAR(capsule_length, bone_length, 1e-9)
            << "part " << binding.part << " does not span its bone";

        // Its segment runs along its own local +Y, so that axis must land on the bone.
        const Vector3 axis = rotate(part.local_orientation, Vector3{0, 1, 0});
        const Vector3 direction = bone * (Scalar(1) / Scalar(bone_length));
        EXPECT_NEAR(double(dot(axis, direction)), 1.0, 1e-9)
            << "part " << binding.part << " is not aimed down its bone";

        // And it is centred on the bone, not hung off one end.
        const Vector3 midpoint = positions[binding.joint] + bone * Scalar(0.5);
        EXPECT_LT(distance(part.local_position, midpoint), 1e-9);
    }
}

TEST(Unit_PhysicsAssembly, ARagdollWeighsWhatItWasAskedTo)
{
    const Humanoid humanoid;
    RagdollProfile profile;
    profile.total_mass = Scalar(70);
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, profile);
    ASSERT_FALSE(rig.assembly.parts.empty());

    const std::vector<EntityId> ids = part_entities(rig.assembly.parts.size());
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(rig.assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    ASSERT_EQ(out.bodies.size(), rig.assembly.parts.size());

    double total = 0;
    double heaviest = 0;
    double lightest = 1e30;
    for (const RigidBodyDescription& body : out.bodies)
    {
        ASSERT_GT(double(body.inv_mass), 0.0);
        const double mass = 1.0 / double(body.inv_mass);
        total += mass;
        heaviest = std::max(heaviest, mass);
        lightest = std::min(lightest, mass);
    }
    EXPECT_NEAR(total, 70.0, 1e-6);

    // Spread by volume at one density, so the parts genuinely differ — a rig whose every
    // part weighed the same would pass the total and be wrong about every limb.
    EXPECT_GT(heaviest, 2.0 * lightest);
}

TEST(Unit_PhysicsAssembly, TheBindOffsetRecoversTheJointPoseFromItsPart)
{
    // The round trip the whole binding exists for. Place every part at exactly the pose
    // instancing gave it, and the resolved targets must reproduce the skeleton's own
    // bind model transforms — which are composed here independently.
    const Humanoid humanoid;
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, RagdollProfile{});
    ASSERT_FALSE(rig.bindings.empty());

    std::vector<Vector3> positions;
    std::vector<Quaternion> rotations;
    bind_pose(humanoid.view, positions, rotations);

    const std::vector<EntityId> ids = part_entities(rig.assembly.parts.size());
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(rig.assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});
    ASSERT_EQ(out.bodies.size(), ids.size());

    StubBodies bodies;
    for (const RigidBodyDescription& body : out.bodies)
        bodies.set_rigid_pose(body.id, body.position, body.orientation);

    std::vector<Animation::RagdollJointTarget> targets;
    const std::size_t written = resolve_ragdoll_targets(rig, bodies, ids.data(), ids.size(),
                                                        Matrix4{}, Scalar(1), targets);
    ASSERT_EQ(written, rig.bindings.size());

    for (const Animation::RagdollJointTarget& target : targets)
    {
        EXPECT_FLOAT_EQ(target.weight, 1.0f);
        const Vector3 recovered = translation_of(target.object_space_transform);
        EXPECT_LT(distance(recovered, positions[target.joint]), 1e-5)
            << "joint " << target.joint << " did not come back where it started";
    }
}

TEST(Unit_PhysicsAssembly, TargetsComeBackInTheCharactersObjectSpace)
{
    // The same round trip with the character somewhere else in the world. The targets
    // must be unchanged, because RagdollBlend blends in object space and a target that
    // carried the character's world transform would drag the pose to the origin.
    const Humanoid humanoid;
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, RagdollProfile{});
    const std::vector<EntityId> ids = part_entities(rig.assembly.parts.size());

    const Vector3 root{Scalar(-40), Scalar(7), Scalar(12)};
    const Quaternion turn = quaternion_axis_angle(Vector3{0, 1, 0}, Scalar(1.1));
    const Matrix4 world_from_object = compose_transform(root, turn, Vector3{1, 1, 1});

    const AssemblyInstantiation out = instantiate_assembly(
        to_view(rig.assembly), ids.data(), ids.size(), root, turn);

    StubBodies bodies;
    for (const RigidBodyDescription& body : out.bodies)
        bodies.set_rigid_pose(body.id, body.position, body.orientation);

    std::vector<Animation::RagdollJointTarget> targets;
    ASSERT_EQ(resolve_ragdoll_targets(rig, bodies, ids.data(), ids.size(), world_from_object,
                                      Scalar(1), targets),
              rig.bindings.size());

    std::vector<Vector3> positions;
    std::vector<Quaternion> rotations;
    bind_pose(humanoid.view, positions, rotations);
    for (const Animation::RagdollJointTarget& target : targets)
        EXPECT_LT(distance(translation_of(target.object_space_transform),
                           positions[target.joint]),
                  1e-4)
            << "joint " << target.joint << " carried the character's world transform";
}

TEST(Unit_PhysicsAssembly, TheTwistRangeIsCentredOnTheTwistTheBindPoseAlreadyHolds)
{
    // Both joint frames are built by the shortest rotation onto the same world axis but
    // in each part's own local space, so they generally differ by a rotation about it —
    // a non-zero twist at bind. A range of [-t, +t] would be centred somewhere the rig
    // has never been, and the limit would fight the bind pose from the first substep.
    const Humanoid humanoid;
    RagdollProfile profile;
    profile.twist_limit = Scalar(0.35);
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, profile);
    ASSERT_FALSE(rig.assembly.joints.empty());

    for (const AssemblyJoint& joint : rig.assembly.joints)
    {
        const AssemblyPart& a = rig.assembly.parts[joint.part_a];
        const AssemblyPart& b = rig.assembly.parts[joint.part_b];
        const Quaternion basis_a =
            mul(a.local_orientation, Physics::joint_frame_from_axis<Scalar>(joint.params.axis_a));
        const Quaternion basis_b =
            mul(b.local_orientation, Physics::joint_frame_from_axis<Scalar>(joint.params.axis_b));
        const Scalar twist =
            Physics::joint_twist_angle<Scalar>(normalize(mul(conjugate(basis_a), basis_b)));

        // Inside the range, and at its centre — the range is symmetric about the bind
        // twist by construction.
        EXPECT_GE(double(twist), double(joint.params.twist_limit.lower) - 1e-9);
        EXPECT_LE(double(twist), double(joint.params.twist_limit.upper) + 1e-9);
        const double centre = 0.5 * (double(joint.params.twist_limit.lower) +
                                     double(joint.params.twist_limit.upper));
        EXPECT_NEAR(double(twist), centre, 1e-9);
        EXPECT_NEAR(double(joint.params.twist_limit.upper) - centre, 0.35, 1e-9);

        // The swing is zero at bind whatever the twist is, because both axes are the
        // same world direction — so its limit needs no such correction.
        EXPECT_TRUE(joint.params.swing_limit.enabled);
        EXPECT_NEAR(double(joint.params.swing_limit.upper), double(profile.swing_limit), 1e-12);
    }
}

TEST(Unit_PhysicsAssembly, EveryPartHangsFromTheNearestAncestorThatHasOne)
{
    const Humanoid humanoid;
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, RagdollProfile{});

    // Every part but one is the child end of exactly one joint, and the one that is not
    // is the root — a ragdoll with two unattached parts is two ragdolls.
    std::vector<int> incoming(rig.assembly.parts.size(), 0);
    for (const AssemblyJoint& joint : rig.assembly.joints)
        ++incoming[joint.part_b];

    int unattached = 0;
    for (const int count : incoming)
    {
        EXPECT_LE(count, 1);
        if (count == 0)
            ++unattached;
    }
    EXPECT_EQ(unattached, 1);
    const int root = humanoid.view.find_joint(Animation::hash_name("root"));
    ASSERT_GE(root, 0);
    EXPECT_EQ(incoming[rig.part_of_joint[std::size_t(root)]], 0)
        << "the root part should hang from nothing";
}

TEST(Unit_PhysicsAssembly, ResolveSkipsAPartTheSimulationHasNoBodyFor)
{
    const Humanoid humanoid;
    const RagdollRig rig = build_ragdoll_rig(humanoid.view, RagdollProfile{});
    const std::vector<EntityId> ids = part_entities(rig.assembly.parts.size());
    const AssemblyInstantiation out = instantiate_assembly(
        to_view(rig.assembly), ids.data(), ids.size(), Vector3{}, Quaternion{});

    StubBodies bodies;
    for (std::size_t i = 1; i < out.bodies.size(); ++i)
        bodies.set_rigid_pose(out.bodies[i].id, out.bodies[i].position,
                              out.bodies[i].orientation);

    std::vector<Animation::RagdollJointTarget> targets;
    // One short, and the missing one is skipped rather than reported at its bind pose:
    // a target the physics did not produce is not a physics target, and blending toward
    // one would drag the animation to the rig's rest pose.
    EXPECT_EQ(resolve_ragdoll_targets(rig, bodies, ids.data(), ids.size(), Matrix4{}, Scalar(1),
                                      targets),
              rig.bindings.size() - 1);
    for (const Animation::RagdollJointTarget& target : targets)
        EXPECT_NE(target.joint, rig.bindings[0].joint);

    // And too few entities is refused outright, for the same reason instancing refuses.
    EXPECT_EQ(resolve_ragdoll_targets(rig, bodies, ids.data(), 1, Matrix4{}, Scalar(1), targets),
              std::size_t(0));
}

TEST(Unit_PhysicsAssembly, ASkeletonWithNoBonesYieldsNoRig)
{
    // A single joint has no children, so no bone, so nothing to simulate. Reported as an
    // empty rig rather than a rig of one degenerate capsule.
    Animation::SkeletonDescription desc;
    Animation::JointDescription only;
    only.name = "root";
    only.parent = -1;
    desc.joints.push_back(only);

    std::vector<std::byte> blob;
    ASSERT_TRUE(Animation::build_skeleton_blob(desc, blob));
    const Animation::SkeletonView view = Animation::load_skeleton_blob(blob.data(), blob.size());

    const RagdollRig rig = build_ragdoll_rig(view, RagdollProfile{});
    EXPECT_TRUE(rig.assembly.parts.empty());
    EXPECT_TRUE(rig.assembly.joints.empty());
    EXPECT_TRUE(rig.bindings.empty());

    // And an empty skeleton is not a crash.
    EXPECT_TRUE(build_ragdoll_rig(Animation::SkeletonView{}, RagdollProfile{})
                    .assembly.parts.empty());
}
