/**************************************************************************/
/* cooked_asset_store.cpp                                                 */
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

#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>

#include <filesystem>
#include <fstream>
#include <system_error>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /** @brief The key's folded hash as sixteen lower-case hexadecimal digits. */
                std::string hash_to_hex(std::uint64_t value)
                {
                    static const char digits[] = "0123456789abcdef";
                    std::string text(16, '0');
                    for (int i = 15; i >= 0; --i)
                    {
                        text[std::size_t(i)] = digits[value & 0xFull];
                        value >>= 4;
                    }
                    return text;
                }
            } // namespace

            bool MemoryCookedAssetStore::contains(const CookedAssetKey& key) const
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                return assets_.find(cooked_asset_key_hash(key)) != assets_.end();
            }

            bool MemoryCookedAssetStore::load(const CookedAssetKey& key,
                                              std::vector<std::byte>& out) const
            {
                out.clear();
                const std::lock_guard<std::mutex> guard(mutex_);
                const auto found = assets_.find(cooked_asset_key_hash(key));
                if (found == assets_.end())
                    return false;
                out = found->second;
                return true;
            }

            bool MemoryCookedAssetStore::store(const CookedAssetKey& key,
                                               const std::vector<std::byte>& bytes)
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                assets_[cooked_asset_key_hash(key)] = bytes;
                return true;
            }

            bool MemoryCookedAssetStore::evict(const CookedAssetKey& key)
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                return assets_.erase(cooked_asset_key_hash(key)) > 0;
            }

            std::size_t MemoryCookedAssetStore::size() const
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                return assets_.size();
            }

            void MemoryCookedAssetStore::clear()
            {
                const std::lock_guard<std::mutex> guard(mutex_);
                assets_.clear();
            }

            FilesystemCookedAssetStore::FilesystemCookedAssetStore(std::string directory)
                : directory_(std::move(directory))
            {
                if (directory_.empty())
                    return;

                // Neither create nor exists is allowed to throw: this constructor runs on
                // an import path, and a read-only cache directory must degrade to "cooks
                // every time" rather than take down the import.
                std::error_code error;
                const std::filesystem::path path(directory_);
                if (!std::filesystem::exists(path, error))
                    std::filesystem::create_directories(path, error);
                usable_ = std::filesystem::is_directory(path, error);
            }

            std::string FilesystemCookedAssetStore::path_for(const CookedAssetKey& key) const
            {
                std::string path = directory_;
                if (!path.empty() && path.back() != '/' && path.back() != '\\')
                    path.push_back('/');
                path += hash_to_hex(cooked_asset_key_hash(key));
                path.push_back('.');
                path += cooked_asset_extension(key.kind);
                return path;
            }

            bool FilesystemCookedAssetStore::contains(const CookedAssetKey& key) const
            {
                if (!usable_)
                    return false;
                std::error_code error;
                return std::filesystem::is_regular_file(std::filesystem::path(path_for(key)),
                                                        error);
            }

            bool FilesystemCookedAssetStore::load(const CookedAssetKey& key,
                                                  std::vector<std::byte>& out) const
            {
                out.clear();
                if (!usable_)
                    return false;

                const std::lock_guard<std::mutex> guard(mutex_);
                std::ifstream file(path_for(key), std::ios::binary | std::ios::ate);
                if (!file)
                    return false;
                const std::streamoff size = file.tellg();
                if (size <= 0)
                    return false;
                file.seekg(0, std::ios::beg);
                out.resize(std::size_t(size));
                file.read(reinterpret_cast<char*>(out.data()), size);
                if (file.gcount() != size)
                {
                    out.clear();
                    return false;
                }
                return true;
            }

            bool FilesystemCookedAssetStore::store(const CookedAssetKey& key,
                                                   const std::vector<std::byte>& bytes)
            {
                if (!usable_ || bytes.empty())
                    return false;

                const std::lock_guard<std::mutex> guard(mutex_);
                // Written to a sibling and renamed into place, so a reader on another
                // thread sees either the whole asset or none of it. A half-written blob
                // that validates its own header is the worst kind of cache entry: it
                // loads, and the geometry it describes is not there.
                const std::string final_path = path_for(key);
                const std::string temporary_path = final_path + ".partial";
                {
                    std::ofstream file(temporary_path, std::ios::binary | std::ios::trunc);
                    if (!file)
                        return false;
                    file.write(reinterpret_cast<const char*>(bytes.data()),
                               std::streamsize(bytes.size()));
                    if (!file)
                        return false;
                }

                std::error_code error;
                std::filesystem::rename(std::filesystem::path(temporary_path),
                                        std::filesystem::path(final_path), error);
                if (!error)
                    return true;

                // Rename onto an existing file fails on some platforms; replace and retry
                // once before giving up, and clean up the partial file either way.
                std::filesystem::remove(std::filesystem::path(final_path), error);
                std::filesystem::rename(std::filesystem::path(temporary_path),
                                        std::filesystem::path(final_path), error);
                if (error)
                {
                    std::error_code ignored;
                    std::filesystem::remove(std::filesystem::path(temporary_path), ignored);
                    return false;
                }
                return true;
            }

            bool FilesystemCookedAssetStore::evict(const CookedAssetKey& key)
            {
                if (!usable_)
                    return false;
                const std::lock_guard<std::mutex> guard(mutex_);
                std::error_code error;
                return std::filesystem::remove(std::filesystem::path(path_for(key)), error);
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
