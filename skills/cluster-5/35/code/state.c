#include <stdio.h>

typedef enum {
    InitialState,
    ActiveGame,
    GameOver,
    MAX_STATES
} state_e;

typedef enum {
    StartEvent,
    MissedMoleEvent,
    HitMoleEvent,
    TimerEndsEvent,
    WaitEvent,
    MAX_EVENTS
} event_e;

state_e state = InitialState;
state_e next_state;
event_e event;

state_e startEventHandler();
state_e missedMoleEventHandler();
state_e hitMoleEventHandler();
state_e timerEndsEventHandler();
state_e waitEventHandler();

event_e read_event();

int points = 0;

int main() {
    while (1) {  // Main loop
        event = read_event();

        if (state == InitialState) {
            if (event == StartEvent) {
                next_state = startEventHandler();
            }
        } else if (state == ActiveGame) {
            if (event == MissedMoleEvent) {
                next_state = missedMoleEventHandler();
                points += 0;
            } else if (event == HitMoleEvent) {
                next_state = hitMoleEventHandler();
                points += 10;
            } else if (event == TimerEndsEvent) {
                next_state = timerEndsEventHandler();
            }
        } else if (state == GameOver) {
            if (event == WaitEvent) {
                next_state = waitEventHandler();
            }
        }

        state = next_state;
    }

    return 0;
}

// Event handler functions
state_e startEventHandler() {
    return ActiveGame;
}

state_e missedMoleEventHandler() {
    return ActiveGame;
}

state_e hitMoleEventHandler() {
    return ActiveGame;
}

state_e timerEndsEventHandler() {
    return GameOver;
}

state_e waitEventHandler() {
    return InitialState;
}

event_e read_event() {
    return StartEvent; 
}
