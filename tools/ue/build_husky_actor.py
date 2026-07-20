"""Build BP_HuskyVisual programmatically (UE 5.4/5.5 Editor Python).

Run with:
  UnrealEditor /abs/path/Blocks.uproject \
    -ExecutePythonScript="/abs/path/build_husky_actor.py" \
    -unattended -stdout -FullStdOutLogOutput

Effect:
  - Create /Game/Vehicle/BP_HuskyVisual (Blueprint of Actor)
  - Set DefaultSceneRoot mobility to Movable
  - Add SkeletalMeshComponent ('CPHuskyMesh') using /Game/Vehicle/CPHusky.CPHusky
  - Add StaticMeshComponent ('TopPanel')      using /Game/Vehicle/CPHusky_TopPanel.CPHusky_TopPanel
  - Disable physics / set collision profile NoCollision on every component
  - Compile + save the asset.

Idempotent: rerunning replaces / refreshes the blueprint instead of duplicating it.
"""

from __future__ import annotations

import unreal


BLUEPRINT_PATH = "/Game/Vehicle/BP_HuskyVisual"
SKELETAL_MESH_PATH = "/Game/Vehicle/CPHusky/CPHusky.CPHusky"
TOP_PANEL_MESH_PATH = "/Game/Vehicle/CPHusky/CPHusky_TopPanel.CPHusky_TopPanel"


def _ensure_blueprint(asset_path: str) -> "unreal.Blueprint":
    pkg_path, _, asset_name = asset_path.rpartition("/")
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()

    existing = unreal.EditorAssetLibrary.load_asset(asset_path)
    if existing is not None:
        unreal.log_warning(f"BP_HuskyVisual already exists, deleting and recreating: {asset_path}")
        unreal.EditorAssetLibrary.delete_asset(asset_path)

    factory = unreal.BlueprintFactory()
    factory.set_editor_property("parent_class", unreal.Actor)
    bp = asset_tools.create_asset(asset_name, pkg_path, unreal.Blueprint, factory)
    if bp is None:
        raise RuntimeError(f"Failed to create blueprint at {asset_path}")
    return bp


def _set_no_collision_no_physics(scs_node) -> None:
    component_template = scs_node.component_template
    try:
        component_template.set_editor_property("simulate_physics", False)
    except Exception:
        pass
    try:
        component_template.set_editor_property("collision_profile_name", "NoCollision")
    except Exception:
        pass
    try:
        component_template.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
    except Exception:
        pass


def _try_set(obj, names, value) -> bool:
    """Set the first editor property that exists on obj. Returns True if any succeeded."""
    last_err = None
    for n in names:
        try:
            obj.set_editor_property(n, value)
            return True
        except Exception as e:
            last_err = e
    if last_err is not None:
        unreal.log_warning(f"set_editor_property tried {names}, all failed: {last_err}")
    return False


def _disable_physics_and_collision(component) -> None:
    _try_set(component, ["b_simulate_physics", "simulate_physics"], False)
    _try_set(component, ["collision_profile_name"], unreal.Name("NoCollision"))
    _try_set(component, ["mobility"], unreal.ComponentMobility.MOVABLE)


def _resolve_template(handle, bp):
    """UE 5.5 exposes data via SubobjectDataBlueprintFunctionLibrary; older builds use the handle directly."""
    data = None
    try:
        # UE 5.5: BlueprintFunctionLibrary version returns FSubobjectData (tuple in Python).
        data = unreal.SubobjectDataBlueprintFunctionLibrary.get_data(handle)
    except Exception:
        pass
    if data is None:
        try:
            data = handle.get_data()
        except Exception:
            data = None
    if data is None:
        raise RuntimeError("SubobjectDataHandle has no underlying data (unsupported UE version).")

    template = None
    try:
        template = unreal.SubobjectDataBlueprintFunctionLibrary.get_object_for_blueprint(data, bp)
    except Exception:
        pass
    if template is None:
        try:
            template = data.get_object_for_blueprint(bp)
        except Exception:
            try:
                template = unreal.SubobjectDataBlueprintFunctionLibrary.get_object(data)
            except Exception:
                try:
                    template = data.get_object()
                except Exception:
                    template = None
    return template


def _add_skeletal_mesh_component(bp, name: str, mesh_path: str):
    mesh = unreal.EditorAssetLibrary.load_asset(mesh_path)
    if mesh is None:
        raise RuntimeError(f"SkeletalMesh not found at {mesh_path}")

    sub = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    handles = sub.k2_gather_subobject_data_for_blueprint(bp)
    root_handle = handles[0]
    add_params = unreal.AddNewSubobjectParams(
        parent_handle=root_handle,
        new_class=unreal.SkeletalMeshComponent,
        blueprint_context=bp,
    )
    new_handle, fail_reason = sub.add_new_subobject(add_params)
    if not fail_reason.is_empty():
        raise RuntimeError(f"Failed to add SkeletalMeshComponent: {fail_reason}")
    sub.rename_subobject(new_handle, unreal.Text(name))

    template = _resolve_template(new_handle, bp)
    if isinstance(template, unreal.SkeletalMeshComponent):
        _try_set(template, ["skeletal_mesh_asset", "skeletal_mesh"], mesh)
        _disable_physics_and_collision(template)
    return new_handle


def _add_static_mesh_component(bp, name: str, mesh_path: str, parent_handle):
    mesh = unreal.EditorAssetLibrary.load_asset(mesh_path)
    if mesh is None:
        raise RuntimeError(f"StaticMesh not found at {mesh_path}")

    sub = unreal.get_engine_subsystem(unreal.SubobjectDataSubsystem)
    add_params = unreal.AddNewSubobjectParams(
        parent_handle=parent_handle,
        new_class=unreal.StaticMeshComponent,
        blueprint_context=bp,
    )
    new_handle, fail_reason = sub.add_new_subobject(add_params)
    if not fail_reason.is_empty():
        raise RuntimeError(f"Failed to add StaticMeshComponent: {fail_reason}")
    sub.rename_subobject(new_handle, unreal.Text(name))

    template = _resolve_template(new_handle, bp)
    if isinstance(template, unreal.StaticMeshComponent):
        _try_set(template, ["static_mesh"], mesh)
        _disable_physics_and_collision(template)
    return new_handle


def main() -> None:
    bp = _ensure_blueprint(BLUEPRINT_PATH)

    skeletal_handle = _add_skeletal_mesh_component(bp, "CPHuskyMesh", SKELETAL_MESH_PATH)
    _add_static_mesh_component(bp, "TopPanel", TOP_PANEL_MESH_PATH, skeletal_handle)

    unreal.EditorAssetLibrary.save_asset(BLUEPRINT_PATH, only_if_is_dirty=False)
    print(f"BP_HuskyVisual saved at {BLUEPRINT_PATH}")


main()
