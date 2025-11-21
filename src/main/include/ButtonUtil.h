#pragma once

/**
 * Rescale trigger input to be more controllable
 * Adds some deadzone and cubes the output
 */
double ConditionRawTriggerInput(double RawTrigVal) noexcept;

/**
 * Rescale joystick input to be more controllable
 * Adds some deadzone and cubes the output
 */
double ConditionRawJoystickInput(double RawJoystickVal, double mixer = 0.75) noexcept;
