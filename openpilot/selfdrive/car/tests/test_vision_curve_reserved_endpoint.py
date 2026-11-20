from openpilot.selfdrive.car.vision_curve_button_transport import BoundedButtonTransport
import pytest


def ledger(display=55,owed=2):
  t=BoundedButtonTransport(display,ceiling_mph=70,minus_interval=.18,plus_interval=.18,press_s=.13,burst=3)
  t.debt[-1]=owed
  return t


def sent(out):
  return out.reason in ('new-pulse','repeat-pulse')


def test_fractional_last_slot_waits_until_new_bend_and_keeps_both_old_credits():
  t=ledger()
  assert not sent(t.update(0,55,52.918))
  assert t.reserve_down_slot and t.down_debt==2
  assert not sent(t.update(1,55,45))
  observed=55
  for i in range(1500):
    out=t.update(2+i/100,observed,70)
    if sent(out):
      assert out.button=='SET+' and observed+t.up_debt<=70
      observed+=1
  assert observed==70 and not t.reserve_down_slot and t.down_debt==2
  for i in range(1600):
    out=t.update(18+i/100,observed,50)
    if sent(out):
      assert out.button=='SET-'
      assert observed-t.down_debt>=50
      observed-=1
  assert observed==52 and t.down_debt==2


@pytest.mark.parametrize(('target','owed','transmitted'),[(52,2,True),(52.001,2,False),
                           (51.9,2,True),(52.2,1,True),(54.8,0,True)])
def test_original_directions_and_unrounded_last_bound(target,owed,transmitted):
  t=ledger(owed=owed)
  assert sent(t.update(0,55,target)) is transmitted


def test_arriving_old_credits_open_matching_headroom_without_time_expiry():
  t=ledger()
  assert not sent(t.update(0.,55,52.9))
  assert not sent(t.update(24.,55,40))
  out=t.update(24.01,54,40)
  assert sent(out) and t.down_debt==2
  assert not sent(t.update(27.,54,40))
  assert not sent(t.update(27.1,52,40,allowed=False))
  assert t.down_debt==0


def test_no_spending_in_driver_override_or_for_control_that_cannot_assert():
  t=ledger()
  for i in range(1000):
    assert not sent(t.update(i/100,55,52.99,allowed=(i%2==0),can_assert=False))
  assert not t.reserve_down_slot and t.down_debt==2


def test_all_new_lost_still_fail_closed_at_original_three():
  t=ledger(display=70)
  for i in range(1000): t.update(i/100,70,45)
  assert t.down_debt==3
  assert not sent(t.update(200.,70,45))
