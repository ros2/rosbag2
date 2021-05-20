# Keyboard handler
Package providing ability to handle keyboard input via simple interface with callbacks. 

## Goal
We need to be able to handle keyboard input in unified way with cross-platform implementation.

##Design requirements:
* Subscribe to keyboard events via callbacks
* Subscriptions work with key modifiers (e.g. Shift+F1)
* Multiple clients can subscribe to the same event

## Design Proposal
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

## Handling cases when client's code subscribed to the key press event got destructed before keyboard handler.
There are two options to properly handle this case:
1. Keep callback handle returning from `KeyboardHandler::add_key_press_callback(..)` and use it 
   in client destructor to delete callback via explicit call to the 
   `KeyboardHandler::delete_key_press_callback(const callback_handle_t & handle)`
2. Use lambda with `weak_ptr` to the client instance as callback. This approach have assumption 
   that client will be instantiated as shared pointer and this shared pointer will be available 
   during callback registration. \
   Here are two different approaches for this option:
   1. Using factory design pattern to create instance of client:
      ```cpp
      KeyboardHandler keyboard_handler;
      std::shared_ptr<Recorder> recorder = Recorder::create();
      recorder->register_callbacks(keyboard_handler);
      ``` 
      where `Recorder` class could be defined as:
      ```cpp
      class Recorder
      {
      public:
        const Recorder & operator=(const Recorder &) = delete;
        Recorder(const Recorder &) = delete;
      
        static std::shared_ptr<Recorder> create()
        {
          auto recorder_shared_ptr = std::shared_ptr<Recorder>(new Recorder);
          recorder_shared_ptr->weak_self_ = recorder_shared_ptr;
          return recorder_shared_ptr;
        }
      
        void register_callbacks(KeyboardHandler & keyboard_handler)
        {
          auto callback = [recorder_weak_ptr = weak_self_](KeyboardHandler::KeyCode key_code) {
            auto recorder_shared_ptr = recorder_weak_ptr.lock();
            if (recorder_shared_ptr) {
              recorder_shared_ptr->callback_func(key_code);
            } else {
              std::cout << "Object for assigned callback was deleted" << std::endl;
            }
          };
          keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP);
        }
      
      private:
        std::weak_ptr<Recorder> weak_self_;
        Recorder();
        void callback_func(KeyboardHandler::KeyCode key_code);
      }
      ```
   2. Using [`shared_from_this()`](https://en.cppreference.com/w/cpp/memory/enable_shared_from_this/enable_shared_from_this). 
      Note in this case client class should be inherited from 
      [`std::enable_shared_from_this`](https://en.cppreference.com/w/cpp/memory/enable_shared_from_this/enable_shared_from_this) 
      e.g.
      ```cpp
      class Player : public std::enable_shared_from_this<Player>
      {
      public:
        Player();
        void register_callbacks(KeyboardHandler & keyboard_handler)
        {
          std::weak_ptr<Player> player_weak_ptr(shared_from_this());
          auto callback = [player_weak_ptr](KeyboardHandler::KeyCode key_code) {
            auto player_shared_ptr = player_weak_ptr.lock();
            if (player_shared_ptr) {
              player_shared_ptr->callback_func(key_code);
            } else {
              std::cout << "Object for assigned callback was deleted" << std::endl;
            }
          };
          keyboard_handler.add_key_press_callback(callback, KeyboardHandler::KeyCode::CURSOR_UP);
        }
      ```
      `Player` class could be instantiated as 
      `std::shared_ptr<Player> player_shared_ptr(new Player());`
   
   
## Handling cases when standard input from terminal or console redirected to the file or stream.
By design keyboard handler rely on the assumption that it will poll on keypress event and then 
readout pressed keys from standard input. When standard input redirected to be read from the 
file or stream, keypress even will not happened and logic inside keyboard handler will be 
ill-formed and could lead to the deadlock. To avoid this scenario keyboard handler perform check 
in constructor if current terminal or console input bind to the real HW and was not redirected to 
be read from file or stream.

Note. Code executing under the `gtest` framework using redirection of the standard input.

From first glance it would be obvious if keyboard handler would throw exception if detected 
that standard input doesn't bind to the real HW. However it will require special handling inside 
client code which will use it. Especially when it will be running under the gtest. It would be 
ok if keyboard handler creating after it's clients. But in cases if for some reason keyboard 
handler should be created before it's clients, whole code which is depend on it is going to be 
skipped during unit test. For instance:
```cpp
  try {
    KeyboardHandler keyboard_handler;
    // .. Some other complicated logic maybe instantiating client in a separate thread 
    std::shared_ptr<Client> client = Client::create();
    client->register_callbacks(keyboard_handler);
    // Use client to test it's internal functionality
  catch (...) {
    // Handle exceptions
  }
```
As you can see it will be impossible to separate exception handling for the case if keyboard 
handler will throw exception when it's running under the `gtest`.

To be able smoothly use keyboard handler and it's dependent clients code under the `gtest` it 
was chosen design approach when keyboard handler not throwing exception on construction if 
standard input doesn't bind to the real HW. Instead of throwing exception keyboard handler will 
go in to the special safe state where he will aware about this situation and keyboard handling 
will be disabled.

Note: By design `KeyboardHandler::add_key_press_callback(..)` API call will return `false` in 
this case to indicate that handling keypress is not possible in this case.   