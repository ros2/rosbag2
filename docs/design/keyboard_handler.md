# Keyboard handler
Package providing ability to handle keyboard input via simple interface with callbacks. 

## Goal
We need to be able to handle keyboard input in unified way with cross-platform implementation.

## Design Proposal
It would be great to have some simple interface to be able to subscribe to the specific key 
press event using callbacks. Also it would useful to have ability to handle events for 
combination of keys pressed in one callback as well as have possibility to subscribe to the 
same event from multiple clients.

The following pseudocode suggests the high level API for aforementioned design proposal.
```cpp
    ClientClass1 client1;
    ClientClass2 client2;
    KeyboardHandler keyboard_handler;
    client1.register_key_press_callbacks(keyboard_handler);
    client2.register_callbacks_for_keyboard_handler(keyboard_handler);
```
Inside client class registering for the callbacks could be organized as simple as:
```cpp
  ClientClass1::register_key_press_callbacks(KeyboardHandler & keyboard_handler) 
  {
    keyboard_handler.add_key_press_callback(callback_fn, KeyboardHandler::KeyCode::CURSOR_UP);
    keyboard_handler.add_key_press_callback(callback_fn, KeyboardHandler::KeyCode::SHIFT_F1);
  }
```
To be able to handle multiple events in one callback function and have ability to distinguish 
them inside, this callback function should have input parameter indicating which key combination 
handling in the current call. For instance it could be implemented as simple as:
```cpp
  void callback_function(KeyboardHandler::KeyCode key_code)
  {
    using KeyCode = KeyboardHandler::KeyCode;
    switch (key_code) {
      case KeyCode::CURSOR_UP:
        std::cout << "callback with key code = CURSOR_UP" << std::endl;
      break;
      case KeyCode::SHIFT_F1:
        std::cout << "callback for pressed key combination = Shift + F1" << std::endl;
      break;
      default:
       std::cout << "callback with key code = " << static_cast<int32_t>(key_code) << std::endl;
      break;
    }
  }
```

## Consideration of using C++ versus Python for cross-platform implementation
At the very early design discussions was proposed to use Python as cross-platform 
implementation for keyboard handling. From the first glance it looks attractive to use Python 
since it's relatively new language with reach built-in utility functions available on multiple 
platforms and OSes.
However after some research and consideration we come to the conclusion that using Python is 
not better than C or C++ for this case. And here is the rational for that:
1. It turns out that there are no built-in and cross-platform utility functions for handling input 
   from keyboard in Python. All what you can find is a third party libraries with similar 
   compile time division for Windows and Unix platforms. Most of the libraries not mature 
   enough and have low quality.
2. Most of the ROS2 code written in C++ and it will be much easy to support and use solution
   written on the same language.
3. Some of the POSIX compatible OSes doesn't have Python interpreter. For instance [QNX for safety](https://blackberry.qnx.com/en/software-solutions/embedded-software/qnx-os-for-safety) 

## Specific of handling input from keyboard on different platforms
Processing of the keyboard handling is differ for different operating systems.
There are two major different approaches:
1. Unix/MacOS aka POSIX "compatible" OSes
2. Windows family OSes

Posix compatible OSes approach based on readout from standard input linked to the terminal.
Pressed key combination represented as sequence of 8 bit values. Number of the 8 bit values in sequence
could vary from pressed key combination and usually in range of 1-8 chars. Note: There are no
'\0' value representing null terminator in providing sequence of bytes. i.e. those byte
sequence could be treated as "C" strings if add null terminator at the end.

In Windows family OSes available mechanism for keyboard handling derived from DOS which is based on
polling "`int kbhit()`" system function call until it will return non zero value indicating that
some key was pressed. After that need to read integer value(s) from the standard input to
determine which key was pressed. Pressed key combinations could be represented as one or two 
integer values. Simple alphanumerical keys represented as single integer. Control keys and specific
function keys represented as two integer values. Note: for some key combinations first integer 
value could contain '\0' value representing null terminator in regular strings. 
i.e it is not possible to treat such sequence of bytes as regular strings.

## Considerations of how to handle cases when client code subscribed to the key press event got destructed before keyboard handler.
TBD.