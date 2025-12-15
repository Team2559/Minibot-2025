#include "subsystems/PIDTuner.h"

void PIDTuner::InitSendable(wpi::SendableBuilder &builder) {
  builder.SetSmartDashboardType("PIDController");
  builder.AddDoubleProperty(
    "p", [this]() {
      return kP;
    }, [this](double kP) {
      this->kP = kP;
      handler({PIDUpdate::PIDTerm::kP, kP, slot});
    }
  );
  builder.AddDoubleProperty(
    "i", [this]() {
      return kI;
    }, [this](double kI) {
      this->kI = kI;
      handler({PIDUpdate::PIDTerm::kI, kI, slot});
    }
  );
  builder.AddDoubleProperty(
    "d", [this]() {
      return kD;
    }, [this](double kD) {
      this->kD = kD;
      handler({PIDUpdate::PIDTerm::kD, kD, slot});
    }
  );
  builder.AddDoubleProperty(
    "f", [this]() {
      return kFF;
    }, [this](double kFF) {
      this->kFF = kFF;
      handler({PIDUpdate::PIDTerm::kFF, kFF, slot});
    }
  );
  builder.AddIntegerProperty(
    "slot", [this]() {
      return slot;
    }, [this](int newSlot) {
      slot = newSlot;
    }
  );
}

uint PIDTuner::Slot() {
  return slot;
}
