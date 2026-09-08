from tools.navigator_staging.verify_checkout import validate_diagnostics_schema


def test_actual_car_output_diagnostics_schema():
  validate_diagnostics_schema()
