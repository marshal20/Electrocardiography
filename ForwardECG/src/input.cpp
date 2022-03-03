#include "input.h"


static InputEvent key_events[256];
static InputState key_states[256];
static InputEvent button_events[256];
static InputState button_states[256];
static double xpos_cur, ypos_cur, xpos_last, ypos_last, xpos_next, ypos_next;
static double scroll_cur, scroll_last, scroll_next;

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
		xpos_cur = xpos_last = 0;
		ypos_cur = ypos_last = 0;
		scroll_cur = scroll_last = scroll_next = 0;
		initialized = true;
	}

	// Clear all events.
	for (int i = 0; i < 256; i++)
	{
		key_events[i] = EVENT_NO_EVENT;
		button_events[i] = EVENT_NO_EVENT;
	}

	// cursor position
	xpos_last = xpos_cur;
	ypos_last = ypos_cur;
	xpos_cur = xpos_next;
	ypos_cur = ypos_next;

	// scroll
	scroll_last = scroll_cur;
	scroll_cur = scroll_next;
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

void Input::cursorPositionCallback(double xpos, double ypos)
{
	xpos_next = xpos;
	ypos_next = ypos;
}

void Input::scrollCallback(double xoffset, double yoffset)
{
	scroll_next += yoffset;
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

// Mouse position

double Input::getCursorXPos()
{
	return xpos_cur;
}

double Input::getCursorYPos()
{
	return ypos_cur;
}

double Input::getCursorXDelta()
{
	return xpos_cur - xpos_last;
}

double Input::getCursorYDelta()
{
	return ypos_cur - ypos_last;
}

// Scroll

double Input::getScrollOffset()
{
	return scroll_cur;
}

double Input::getScrollDelta()
{
	return scroll_cur - scroll_last;
}
