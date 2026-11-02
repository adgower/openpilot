def test_real_complete_factory_selects_new_scope_only_for_existing_opt_in():
  from openpilot.selfdrive.car.tracking_context import create_tracking_trial
  from opendbc.car.ford.tests.test_unwind_allocation import make as make_root

  cp = make_root().CP

  def selected(root):
    seen = set()
    flags = []

    def visit(obj):
      if obj is None or id(obj) in seen:
        return
      seen.add(id(obj))
      if 'curve_scope_extension_enabled' in vars(obj):
        flags.append(vars(obj)['curve_scope_extension_enabled'])
      for name in ('reference', 'baseline', 'installed_baseline', 'upper_reference', 'stock', 'unchanged'):
        visit(vars(obj).get(name))

    visit(root)
    return any(flags)

  assert selected(create_tracking_trial(cp, enabled=True, roll_mapping_enabled=True, high_speed_enabled=True))
  assert not selected(create_tracking_trial(cp, enabled=True, roll_mapping_enabled=True, high_speed_enabled=False))
  assert create_tracking_trial(cp, enabled=False, roll_mapping_enabled=True, high_speed_enabled=True) is None
