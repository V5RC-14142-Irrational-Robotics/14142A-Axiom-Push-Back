#ifndef STATEMACHINE_H
#define STATEMACHINE_H

template <typename State>
class StateMachine
{
public:
    explicit StateMachine(State start) : _current(start), _previous(start) {}

    void set(State next)
    {
        _previous = _current;
        _current = next;
    }

    State get() const { return _current; }
    State prev() const { return _previous; }

    bool changed() const { return _current != _previous; }

private:
    State _current;
    State _previous;
};

#endif
