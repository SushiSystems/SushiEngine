/**************************************************************************/
/* instantiation_plan.cpp                                                 */
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

/**
 * @file instantiation_plan.cpp
 * @brief The pure decision that turns a glTF node graph and its settings into entities.
 *
 * The translation unit `docs/design/model_import.md` §3 places every hard import decision in:
 * which node becomes which entity, when a node splits, how a dropped pivot folds its transform
 * into its children, and how a name collision resolves. It links no device and no editor, which
 * is what makes all of that testable on a machine with no GPU.
 *
 * Empty of definitions until the planner is written. The module publishes the `.meta` sidecar
 * from `import_settings_io.cpp` in the meantime.
 */
