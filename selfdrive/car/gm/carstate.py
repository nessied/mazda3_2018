import copy
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.gm.values import DBC, AccState, CanBus, STEER_THRESHOLD

TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
STANDSTILL_THRESHOLD = 10 * 0.0311 * CV.KPH_TO_MS


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.cluster_min_speed = CV.KPH_TO_MS / 2.

    self.loopback_lka_steering_cmd_updated = False
    self.loopback_lka_steering_cmd_ts_nanos = 0
    self.pt_lka_steering_cmd_counter = 0
    self.cam_lka_steering_cmd_counter = 0
    self.buttons_counter = 0

  def update(self, pt_cp, cam_cp, loopback_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]['ACCButtons']
    self.pscm_status = copy.copy(pt_cp.vl["PSCMStatus"])
    # This is to avoid a fault where you engage while still moving backwards after shifting to D.
    # An Equinox has been seen with an unsupported status (3), so only check if either wheel is in reverse (2)
    self.moving_backward = False

    # Variables used for avoiding LKAS faults
    self.loopback_lka_steering_cmd_updated = len(loopback_cp.vl_all["ASCMLKASteeringCmd"]["RollingCounter"]) > 0
    if self.loopback_lka_steering_cmd_updated:
      self.loopback_lka_steering_cmd_ts_nanos = loopback_cp.ts_nanos["ASCMLKASteeringCmd"]["RollingCounter"]
    if self.CP.networkLocation == NetworkLocation.fwdCamera:
      self.pt_lka_steering_cmd_counter = pt_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
      self.cam_lka_steering_cmd_counter = cam_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # sample rear wheel speeds, standstill=True if ECM allows engagement with brake
    ret.standstill = ret.wheelSpeeds.rl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD


    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))

    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition']
    ret.brakePressed = ret.brake >= 10

    ret.gas = pt_cp.vl["AcceleratorPedal"]['AcceleratorPedal']
    ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATBDTorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]['LKATorqueDeliveredStatus']
    ret.steerFaultTemporary = self.lkas_status == 2
    ret.steerFaultPermanent = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (cam_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    cam_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    cam_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    cam_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = cam_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = cam_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = cam_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    ret.parkingBrake = False
    ret.cruiseState.available = True
    ret.espDisabled = False
    ret.accFaulted = False

    ret.cruiseState.enabled = True
    ret.cruiseState.standstill = True
    ret.cruiseState.speed = 20 * CV.KPH_TO_MS
    ret.stockAeb = False
    ret.cruiseState.nonAdaptive = False

    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    messages = []
    messages += [
      ("BCMDoorBeltStatus", 0),
      ("ASCMActiveCruiseControlStatus", 0),
      ("ASCMLKASteeringCmd", 0),
      ("BCMTurnSignals", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ("EBCMBrakePedalPosition", 0),
      ("AcceleratorPedal", 0),
      ("ASCMSteeringButton", 0),
      ("PSCMSteeringAngle", 0),
      ("EBCMWheelSpdFront", 0),
      ("EBCMWheelSpdRear", 0),
      ("ECMPRDNL", 0),
      ("PSCMStatus", 0),
      ("ASCMLKASteeringCmd", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    messages = [
      ("ASCMLKASteeringCmd", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus.LOOPBACK)
