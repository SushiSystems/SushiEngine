/**************************************************************************/
/* interop.hpp                                                            */
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

#pragma once

/**
 * @file interop.hpp
 * @brief Renderer memory another API can address without a copy.
 *
 * The interop mandate in one object: a buffer the renderer's shaders read as an ordinary
 * storage buffer, whose backing allocation is *also* reachable by SushiRuntime's SYCL
 * device, so simulation output (a weather grid, particle state, soft-body positions)
 * becomes renderable by being written rather than by being uploaded. The match key is
 * @c DeviceInfo::uuid — the same UUID @c RenderDeviceDesc::required_uuid already selects
 * the graphics device by — because two APIs share memory only when they are on one
 * physical device.
 *
 * The direction of the seam is deliberate. The renderer *exports*: it allocates, and
 * hands out an OS handle plus the sizes an importer needs. The importing half belongs to
 * whoever owns the other API, which for SushiRuntime means the runtime, because
 * SushiEngine depends on SushiRuntime and never the reverse. Nothing here includes a
 * Vulkan, SYCL, or platform header, so a consumer of this file inherits neither.
 */

#include <array>
#include <cstdint>
#include <memory>

#include <SushiEngine/render/rhi/device.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /** @brief Which flavour of OS handle an exported allocation is named by. */
        enum class InteropHandleKind : std::uint32_t
        {
            None,        /**< Nothing was exported. */
            OpaqueWin32, /**< A Win32 HANDLE (NT handle), owned by the exporting buffer. */
            OpaqueFd,    /**< A POSIX file descriptor, owned by the importer once returned. */
        };

        /**
         * @brief Everything an importing API needs to address an exported allocation.
         *
         * @c allocation_size is the size of the *allocation*, which the driver may have
         * rounded up from the requested buffer size; an importer must allocate against that
         * number, not against @c size_bytes, or the import is rejected.
         */
        struct InteropMemoryHandle
        {
            InteropHandleKind kind = InteropHandleKind::None;

            /**
             * @brief The handle value: a Win32 HANDLE or a file descriptor, widened.
             *
             * Zero when nothing was exported. On Win32 the handle stays owned by the buffer
             * and is closed with it, so an importer must not close it; on POSIX the returned
             * descriptor is a fresh duplicate the importer owns and must close.
             */
            std::uint64_t value = 0;

            std::uint64_t allocation_size = 0;   /**< Bytes the driver actually allocated. */
            std::uint64_t memory_offset = 0;     /**< Where the buffer starts in the allocation. */
            std::array<std::uint8_t, 16> device_uuid{}; /**< The device both sides must be on. */
        };

        /** @brief What an interop buffer is asked to be. */
        struct InteropBufferDesc
        {
            std::uint64_t size_bytes = 0;

            /**
             * @brief Whether the renderer's shaders may take the buffer's device address.
             *
             * Off by default: an address is only needed by a shader that walks the buffer as
             * a pointer rather than binding it as a descriptor.
             */
            bool device_address = false;
        };

        /**
         * @brief A device-local buffer whose memory a second API can import.
         *
         * Renderer-side it is an ordinary storage buffer (the native handle is what a pass
         * binds); simulation-side it is an OS handle to import. Owning the object owns the
         * allocation, so releasing it after the other API has imported it is a use-after-free
         * on that side — lifetime belongs to whoever created it.
         */
        class IInteropBuffer
        {
            public:
                virtual ~IInteropBuffer() = default;

                /** @brief Bytes the buffer was created to hold. */
                virtual std::uint64_t size_bytes() const noexcept = 0;

                /**
                 * @brief The handle and sizes the other API imports with.
                 * @return The export record; @c kind is @c None if the export failed.
                 */
                virtual InteropMemoryHandle memory_handle() const noexcept = 0;

                /**
                 * @brief The backend buffer handle, for a pass that binds it.
                 *
                 * A @c VkBuffer widened to an integer, on the same footing as
                 * @c NativeDeviceHandles: this header stays API-neutral and the one consumer
                 * that is intrinsically Vulkan reinterprets it.
                 *
                 * @return The native buffer handle.
                 */
                virtual std::uint64_t native_buffer() const noexcept = 0;
        };

        /**
         * @brief Allocates a buffer the given device can share with another API.
         *
         * @param device The live render device the allocation belongs to.
         * @param desc   Size and usage the buffer must satisfy.
         * @return The buffer, or nullptr when the device offers no external memory (in which
         *         case the caller's fallback is an ordinary upload — no path depends on this).
         */
        std::unique_ptr<IInteropBuffer> create_interop_buffer(IRenderDevice& device,
                                                              const InteropBufferDesc& desc);
    } // namespace Render
} // namespace SushiEngine
