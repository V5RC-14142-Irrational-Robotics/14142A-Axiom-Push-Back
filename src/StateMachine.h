// StateMachine.h
#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/// A simple enum‐based finite state machine.
template <typename State>
class StateMachine {
public:
  StateMachine(State start) : _current(start), _previous(start) {}

  /// Transition to a new state.
  void set(State next) {
    _previous = _current;
    _current  = next;
  }

  State get() const { return _current; }
  State prev() const { return _previous; }

private:
  State _current;
  State _previous;
};

#endif  // STATEMACHINE_H
