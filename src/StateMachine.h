#ifndef STATEMACHINE_H
#define STATEMACHINE_H

template <typename State>
class StateMachine
{
public:
    StateMachine(State start) : _current(start), _previous(start) {}
    // switch states
    void set(State next)
    {
        _previous = _current;
        _current = next;
    }

    State get() const { return _current; }
    State prev() const { return _previous; }

private:
    State _current;
    State _previous;
};

#endif
