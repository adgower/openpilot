"""Pure card-to-strategy seam; no sockets, vehicle access or transmission."""
def feed_tracking_context(strategy, *, now_ns, state_ns, control_ns, control_valid,
                          model_ns, model_valid, model, state):
  # Missing/unknown context disables the *extra* correction only.
  lane_change = (model is None or str(model.meta.laneChangeState) != 'off'
                 or state.leftBlinker or state.rightBlinker)
  strategy.set_context(now_ns=now_ns,state_ns=state_ns,control_ns=control_ns,
                       model_ns=model_ns,model_valid=bool(control_valid and model_valid),lane_change=lane_change)
