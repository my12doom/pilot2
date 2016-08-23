#What is this?

YAL(Yet Another Link) is a lightweight packet based message marshalling protocol for realtime controlling and streaming purpose.

It can send C-structs over packet based data channels, with or without (limited, for realtime's sake) transmission control, do "remote procedure call"s, or stream high bandwidth tight time limitation videos.

#Types of message
## Time-Critical message, without tranmission control, also the basic message.
e.g. Remote controller stick data.

## Control message, limited tranmission control
e.g. takeoff command, return to launch command, parameters getting/setting.

## Remote procedure calls
based on control message, but with guaranteed oppositing side response.

## Multimedia streaming, with FEC