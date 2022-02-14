#include "input.h"


static InputEvent key_events[256];
static InputState key_states[256];
static InputEvent button_events[256];
static InputState button_states[256];

void Input::newFrame()
{
	// Initialize.
	static bool initialized = false;
	if (!initialized)
	{
		for (int i = 0; i < 256; i++)
		{
			key_states[i] = STATE_UP;
			button_states[i] = STATE_UP;
		}
		initialized = true;
	}
	// Clear all events.
	for (int i = 0; i < 256; i++)
	{
		key_events[i] = EVENT_NO_EVENT;
		button_events[i] = EVENT_NO_EVENT;
	}
}

void Input::handleKeyEvent(uint8_t key, InputEvent event)
{
	key_events[key] = event;
	if (event == EVENT_PRESS)
		key_states[key] = STATE_DOWN;
	else if (event == EVENT_RELEASE)
		key_states[key] = STATE_UP;
}

void Input::handleButtonEvent(uint8_t button, InputEvent event)
{
	button_events[button] = event;
	if (event == EVENT_PRESS)
		button_states[button] = STATE_DOWN;
	else if (event == EVENT_RELEASE)
		button_states[button] = STATE_UP;
}

// Keys.

InputEvent Input::keyEvent(uint8_t key)
{
	return key_events[key];
}

InputState Input::keyState(uint8_t key)
{
	return key_states[key];
}

bool Input::isKeyPressed(uint8_t key)
{
	return keyEvent(key) == EVENT_PRESS;
}

bool Input::isKeyReleased(uint8_t key)
{
	return keyEvent(key) == EVENT_RELEASE;
}

bool Input::isKeyDown(uint8_t key)
{
	return keyState(key) == STATE_DOWN;
}

bool Input::isKeyUp(uint8_t key)
{
	return keyState(key) == STATE_UP;
}

// Buttons.

InputEvent Input::buttonEvent(uint8_t button)
{
	return button_events[button];
}

InputState Input::buttonState(uint8_t button)
{
	return button_states[button];
}

bool Input::isButtonPressed(uint8_t button)
{
	return buttonEvent(button) == EVENT_PRESS;
}

bool Input::isButtonReleased(uint8_t button)
{
	return buttonEvent(button) == EVENT_RELEASE;
}

bool Input::isButtonDown(uint8_t button)
{
	return buttonState(button) == STATE_DOWN;
}

bool Input::isButtonUp(uint8_t button)
{
	return buttonState(button) == STATE_UP;
}
