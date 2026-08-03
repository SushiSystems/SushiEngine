/**************************************************************************/
/* graph.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_DSP_GRAPH_HPP
#define SUSHIENGINE_AUDIO_DSP_GRAPH_HPP

/**
 * @file graph.hpp
 * @brief The block processing graph: a DAG of nodes pulled one fixed block at a time.
 *
 * A @ref BlockGraph is a directed graph of @ref Node processors joined by mono
 * sample ports. `prepare()` linearizes it into a processing order by a topological
 * sort (Kahn's algorithm) computed off the audio thread, allocates one persistent
 * buffer per output port, and resolves every input port to the buffer it reads;
 * `process()` then walks that fixed order once per block and is allocation-,
 * lock-, and syscall-free — safe on the audio thread.
 *
 * **Feedback** (a reverb comb, a delay line's return) is expressed by marking a
 * connection as a feedback edge: it is excluded from the ordering, and because output
 * buffers persist between blocks, the consumer reads the producer's output as it was
 * left at the end of the *previous* block — a one-block z⁻¹, exactly the delay a
 * feedback loop needs to be computable in a single forward pass.
 *
 * This is the S1 base: mono ports, one source per input (sum with a @ref MixNode),
 * a dedicated buffer per output. Buffer pooling and sub-block splitting at parameter
 * boundaries (§3.1) are later refinements that do not change this surface.
 */

#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A processor with a fixed number of mono input and output ports.
             *
             * Derive and implement @ref process. @ref prepare is where a node allocates
             * and sizes its own state (it runs off the audio thread); @ref process must
             * not allocate. Each output port is a distinct buffer; a node must not write
             * an output while reading an input that aliases it *out of order* — the
             * built-in nodes process sample-by-sample, which keeps feedback self-loops
             * safe.
             */
            class Node
            {
                public:
                    /**
                     * @brief Constructs a node with a fixed port count.
                     * @param input_ports  Number of mono input ports.
                     * @param output_ports Number of mono output ports.
                     */
                    Node(int input_ports, int output_ports) noexcept
                        : input_ports_(input_ports), output_ports_(output_ports)
                    {
                    }

                    virtual ~Node() = default;

                    /** @brief The node's input port count. */
                    int input_port_count() const noexcept { return input_ports_; }

                    /** @brief The node's output port count. */
                    int output_port_count() const noexcept { return output_ports_; }

                    /**
                     * @brief Allocates and sizes state for the coming run (off the audio thread).
                     * @param sample_rate      The stream sample rate in Hz.
                     * @param max_block_frames The largest block @ref process will be given.
                     */
                    virtual void prepare(double sample_rate, int max_block_frames)
                    {
                        (void)sample_rate;
                        (void)max_block_frames;
                    }

                    /** @brief Clears any internal state (filter memory, phase, etc.). */
                    virtual void reset() noexcept {}

                    /**
                     * @brief Renders one block. Allocation-, lock-, and syscall-free.
                     * @param inputs      Array of @ref input_port_count read buffers (each @p frame_count long).
                     * @param outputs     Array of @ref output_port_count write buffers (each @p frame_count long).
                     * @param frame_count Number of samples this block.
                     */
                    virtual void process(const float* const* inputs, float* const* outputs,
                                         int frame_count) noexcept = 0;

                private:
                    int input_ports_;
                    int output_ports_;
            };

            /** @brief Opaque handle to a node inside a @ref BlockGraph (its insertion index). */
            using NodeId = std::size_t;

            /**
             * @brief A topologically-scheduled graph of @ref Node processors.
             *
             * Build it by @ref add_node and @ref connect, pick the final tap with
             * @ref set_graph_output, then @ref prepare once and @ref process per block.
             * After @ref prepare, @ref output_buffer points at the designated output's
             * samples for the last processed block.
             */
            class BlockGraph
            {
                public:
                    /**
                     * @brief Adds a node and returns its handle.
                     * @param node The node to take ownership of.
                     * @return The handle used to connect and address the node.
                     */
                    NodeId add_node(std::unique_ptr<Node> node)
                    {
                        nodes_.push_back(std::move(node));
                        return nodes_.size() - 1;
                    }

                    /**
                     * @brief Connects an output port to an input port.
                     *
                     * One source per input port (a later connection to the same input
                     * replaces the earlier); sum multiple sources with a @ref MixNode.
                     * A @p feedback edge is excluded from scheduling and reads the
                     * source's previous-block output (a one-block z⁻¹).
                     *
                     * @param source        The producing node.
                     * @param source_port   Its output port index.
                     * @param destination   The consuming node.
                     * @param dest_port     Its input port index.
                     * @param feedback      True to make this a one-block feedback edge.
                     */
                    void connect(NodeId source, int source_port, NodeId destination, int dest_port,
                                 bool feedback = false)
                    {
                        connections_.push_back(
                            Connection{source, source_port, destination, dest_port, feedback});
                    }

                    /**
                     * @brief Designates which output port is the graph's final tap.
                     * @param node The node whose output is the graph output.
                     * @param port Its output port index.
                     */
                    void set_graph_output(NodeId node, int port = 0) noexcept
                    {
                        output_node_ = node;
                        output_port_ = port;
                    }

                    /**
                     * @brief Linearizes the graph and allocates buffers for a run.
                     *
                     * Computes the processing order by Kahn topological sort over the
                     * non-feedback edges, allocates one @p max_block_frames buffer per
                     * output port, resolves each input port to the buffer it reads (or a
                     * shared zero buffer if unconnected), and prepares every node. Call
                     * once before the first @ref process and again if the topology
                     * changes.
                     *
                     * @param sample_rate      The stream sample rate in Hz.
                     * @param max_block_frames The largest block @ref process will be given.
                     */
                    void prepare(double sample_rate, int max_block_frames)
                    {
                        sample_rate_ = sample_rate;
                        max_block_ = max_block_frames;

                        const std::size_t count = nodes_.size();
                        out_buffers_.assign(count, {});
                        out_ptrs_.assign(count, {});
                        in_ptrs_.assign(count, {});
                        zero_buffer_.assign(static_cast<std::size_t>(max_block_), 0.0f);

                        // One persistent, zero-initialized buffer per output port.
                        for (std::size_t n = 0; n < count; ++n)
                        {
                            const int outs = nodes_[n]->output_port_count();
                            out_buffers_[n].resize(static_cast<std::size_t>(outs));
                            out_ptrs_[n].resize(static_cast<std::size_t>(outs));
                            for (int p = 0; p < outs; ++p)
                            {
                                out_buffers_[n][static_cast<std::size_t>(p)].assign(
                                    static_cast<std::size_t>(max_block_), 0.0f);
                                out_ptrs_[n][static_cast<std::size_t>(p)] =
                                    out_buffers_[n][static_cast<std::size_t>(p)].data();
                            }

                            // Every input defaults to the shared zero buffer.
                            const int ins = nodes_[n]->input_port_count();
                            in_ptrs_[n].assign(static_cast<std::size_t>(ins), zero_buffer_.data());
                        }

                        // Point each connected input at its source output buffer.
                        for (const Connection& c : connections_)
                        {
                            if (c.dest_port >= 0 &&
                                c.dest_port < nodes_[c.destination]->input_port_count() &&
                                c.source_port >= 0 &&
                                c.source_port < nodes_[c.source]->output_port_count())
                            {
                                in_ptrs_[c.destination][static_cast<std::size_t>(c.dest_port)] =
                                    out_ptrs_[c.source][static_cast<std::size_t>(c.source_port)];
                            }
                        }

                        compute_order();

                        for (std::size_t n = 0; n < count; ++n)
                            nodes_[n]->prepare(sample_rate_, max_block_);
                    }

                    /**
                     * @brief Processes one block through the scheduled order.
                     * @param frame_count Number of samples (clamped to the prepared maximum).
                     */
                    void process(int frame_count) noexcept
                    {
                        if (frame_count > max_block_)
                            frame_count = max_block_;
                        for (NodeId n : order_)
                        {
                            const float* const* in = in_ptrs_[n].empty()
                                                         ? nullptr
                                                         : in_ptrs_[n].data();
                            float* const* out = out_ptrs_[n].empty()
                                                    ? nullptr
                                                    : out_ptrs_[n].data();
                            nodes_[n]->process(in, out, frame_count);
                        }
                    }

                    /** @brief Clears every node's internal state and zeroes all buffers. */
                    void reset() noexcept
                    {
                        for (std::unique_ptr<Node>& node : nodes_)
                            node->reset();
                        for (std::vector<std::vector<float>>& node_bufs : out_buffers_)
                            for (std::vector<float>& buf : node_bufs)
                                std::fill(buf.begin(), buf.end(), 0.0f);
                    }

                    /** @brief The designated output buffer of the last processed block. */
                    const float* output_buffer() const noexcept
                    {
                        if (output_node_ >= out_ptrs_.size())
                            return nullptr;
                        const std::vector<float*>& ports = out_ptrs_[output_node_];
                        if (output_port_ < 0 ||
                            static_cast<std::size_t>(output_port_) >= ports.size())
                            return nullptr;
                        return ports[static_cast<std::size_t>(output_port_)];
                    }

                    /** @brief The number of nodes in the graph. */
                    std::size_t node_count() const noexcept { return nodes_.size(); }

                    /**
                     * @brief Typed access to a node, for parameter updates.
                     * @tparam T The concrete node type.
                     * @param id The node handle.
                     * @return The node as @p T*, or nullptr if the type does not match.
                     */
                    template <typename T>
                    T* node_as(NodeId id) noexcept
                    {
                        if (id >= nodes_.size())
                            return nullptr;
                        return dynamic_cast<T*>(nodes_[id].get());
                    }

                    /** @brief The processing order computed by @ref prepare (for diagnostics/tests). */
                    const std::vector<NodeId>& order() const noexcept { return order_; }

                private:
                    struct Connection
                    {
                        NodeId source;
                        int source_port;
                        NodeId destination;
                        int dest_port;
                        bool feedback;
                    };

                    void compute_order()
                    {
                        const std::size_t count = nodes_.size();
                        std::vector<int> in_degree(count, 0);
                        std::vector<std::vector<NodeId>> adjacency(count);

                        // Build the DAG from non-feedback edges only.
                        for (const Connection& c : connections_)
                        {
                            if (c.feedback || c.source == c.destination)
                                continue;
                            adjacency[c.source].push_back(c.destination);
                            ++in_degree[c.destination];
                        }

                        order_.clear();
                        order_.reserve(count);
                        std::vector<NodeId> ready;
                        for (std::size_t n = 0; n < count; ++n)
                            if (in_degree[n] == 0)
                                ready.push_back(n);

                        while (!ready.empty())
                        {
                            const NodeId n = ready.back();
                            ready.pop_back();
                            order_.push_back(n);
                            for (NodeId m : adjacency[n])
                                if (--in_degree[m] == 0)
                                    ready.push_back(m);
                        }

                        // A cycle with no feedback cut leaves nodes unscheduled; append
                        // them in index order so the graph still runs (best effort).
                        if (order_.size() != count)
                            for (std::size_t n = 0; n < count; ++n)
                                if (in_degree[n] > 0)
                                    order_.push_back(n);
                    }

                    std::vector<std::unique_ptr<Node>> nodes_;
                    std::vector<Connection> connections_;

                    std::vector<NodeId> order_;
                    std::vector<std::vector<std::vector<float>>> out_buffers_; // [node][port][sample]
                    std::vector<std::vector<float*>> out_ptrs_;                // [node][port]
                    std::vector<std::vector<const float*>> in_ptrs_;           // [node][port]
                    std::vector<float> zero_buffer_;

                    double sample_rate_ = 48000.0;
                    int max_block_ = 0;
                    NodeId output_node_ = 0;
                    int output_port_ = 0;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
