#pragma once
#include <stdint.h>

// The user has to call Input::handle_key_event whenever a key event occur, and call new_frame.
// new_frame should be called first, then handle_key_event.


enum InputEvent { EVENT_PRESS = 1, EVENT_RELEASE, EVENT_NO_EVENT };
enum InputState { STATE_DOWN = 1, STATE_UP };

class Input
{
public:
	static void newFrame();
	static void handleKeyEvent(uint8_t key, InputEvent event);
	static void handleButtonEvent(uint8_t button, InputEvent event);

	// Keys
	static InputEvent keyEvent(uint8_t key);
	static InputState keyState(uint8_t key);
	static bool isKeyPressed(uint8_t key);
	static bool isKeyReleased(uint8_t key);
	static bool isKeyDown(uint8_t key);
	static bool isKeyUp(uint8_t key);

	// Buttons
	static InputEvent buttonEvent(uint8_t button);
	static InputState buttonState(uint8_t button);
	static bool isButtonPressed(uint8_t button);
	static bool isButtonReleased(uint8_t button);
	static bool isButtonDown(uint8_t button);
	static bool isButtonUp(uint8_t button);
};

