
/* Copyright (c) 2005-2007, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#ifndef EQ_COMMANDS_H
#define EQ_COMMANDS_H

#include <eq/net/commands.h>

namespace eq
{
    enum ServerCommand
    {
        CMD_SERVER_CHOOSE_CONFIG        = eqNet::CMD_NODE_CUSTOM,
        REQ_SERVER_CHOOSE_CONFIG,
        CMD_SERVER_CHOOSE_CONFIG_REPLY,
        CMD_SERVER_CREATE_CONFIG,
        CMD_SERVER_DESTROY_CONFIG,
        CMD_SERVER_RELEASE_CONFIG,
        REQ_SERVER_RELEASE_CONFIG,
        CMD_SERVER_RELEASE_CONFIG_REPLY,
        CMD_SERVER_INIT_CONFIG,
        REQ_SERVER_INIT_CONFIG, // REQ must follow CMD
        CMD_SERVER_CUSTOM
    };

    enum ClientCommand
    {
        CMD_CLIENT_EXIT               = CMD_SERVER_CUSTOM,
        REQ_CLIENT_EXIT,
        CMD_CLIENT_CUSTOM
    };

    enum ConfigCommand
    {
        CMD_CONFIG_INIT                 = eqNet::CMD_SESSION_CUSTOM,
        REQ_CONFIG_INIT, // REQ must always follow CMD
        CMD_CONFIG_INIT_REPLY,
        REQ_CONFIG_INIT_REPLY,
        CMD_CONFIG_EXIT,
        REQ_CONFIG_EXIT, // REQ must always follow CMD
        CMD_CONFIG_EXIT_REPLY,
        REQ_CONFIG_EXIT_REPLY,
        CMD_CONFIG_CREATE_NODE,
        CMD_CONFIG_DESTROY_NODE,
        CMD_CONFIG_START_FRAME,
        REQ_CONFIG_START_FRAME, // REQ must always follow CMD
        CMD_CONFIG_START_FRAME_REPLY,
        CMD_CONFIG_END_FRAME,
        REQ_CONFIG_END_FRAME, // REQ must always follow CMD
        CMD_CONFIG_END_FRAME_REPLY,
        CMD_CONFIG_FINISH_FRAMES,
        REQ_CONFIG_FINISH_FRAMES, // REQ must always follow CMD
        CMD_CONFIG_FINISH_FRAMES_REPLY,
        CMD_CONFIG_EVENT,
        CMD_CONFIG_CUSTOM
    };

    enum NodeCommand
    {
        CMD_NODE_INIT = eqNet::CMD_OBJECT_CUSTOM,
        REQ_NODE_INIT,
        CMD_NODE_INIT_REPLY,
        CMD_NODE_EXIT,
        REQ_NODE_EXIT,
        CMD_NODE_EXIT_REPLY,
        CMD_NODE_CREATE_PIPE,
        CMD_NODE_DESTROY_PIPE,
        CMD_NODE_CUSTOM
    };

    enum PipeCommand
    {
        CMD_PIPE_INIT = eqNet::CMD_OBJECT_CUSTOM,
        REQ_PIPE_INIT,
        CMD_PIPE_INIT_REPLY,
        CMD_PIPE_EXIT,
        REQ_PIPE_EXIT,
        CMD_PIPE_EXIT_REPLY, 
        CMD_PIPE_UPDATE,
        REQ_PIPE_UPDATE,
        CMD_PIPE_CREATE_WINDOW,
        CMD_PIPE_DESTROY_WINDOW,
        CMD_PIPE_FRAME_SYNC,
        REQ_PIPE_FRAME_SYNC,
        CMD_PIPE_FRAME_SYNC_REPLY,
        CMD_PIPE_CUSTOM
    };

    enum WindowCommand
    {
        CMD_WINDOW_INIT = eqNet::CMD_OBJECT_CUSTOM,
        REQ_WINDOW_INIT,
        CMD_WINDOW_INIT_REPLY,
        CMD_WINDOW_EXIT,
        REQ_WINDOW_EXIT,
        CMD_WINDOW_EXIT_REPLY,
        CMD_WINDOW_CREATE_CHANNEL,
        CMD_WINDOW_DESTROY_CHANNEL,
        CMD_WINDOW_SET_PVP,
        REQ_WINDOW_SET_PVP,
        CMD_WINDOW_FINISH,
        REQ_WINDOW_FINISH,
        CMD_WINDOW_BARRIER,
        REQ_WINDOW_BARRIER,
        CMD_WINDOW_SWAP,
        REQ_WINDOW_SWAP,
        CMD_WINDOW_STARTFRAME,
        REQ_WINDOW_STARTFRAME,
        CMD_WINDOW_ENDFRAME,
        REQ_WINDOW_ENDFRAME,
        CMD_WINDOW_CUSTOM
    };

    enum ChannelCommand
    {
        CMD_CHANNEL_INIT = eqNet::CMD_OBJECT_CUSTOM,
        REQ_CHANNEL_INIT,
        CMD_CHANNEL_INIT_REPLY,
        CMD_CHANNEL_EXIT,
        REQ_CHANNEL_EXIT,
        CMD_CHANNEL_EXIT_REPLY,
        CMD_CHANNEL_SET_NEARFAR,
        REQ_CHANNEL_SET_NEARFAR,
        CMD_CHANNEL_CLEAR,
        REQ_CHANNEL_CLEAR,
        CMD_CHANNEL_DRAW,
        REQ_CHANNEL_DRAW,
        CMD_CHANNEL_ASSEMBLE,
        REQ_CHANNEL_ASSEMBLE,
        CMD_CHANNEL_READBACK,
        REQ_CHANNEL_READBACK,
        CMD_CHANNEL_TRANSMIT,
        REQ_CHANNEL_TRANSMIT,
        CMD_CHANNEL_CUSTOM
    };

    enum FrameDataCommand
    {
        CMD_FRAMEDATA_TRANSMIT = eqNet::CMD_OBJECT_CUSTOM,
        CMD_FRAMEDATA_READY,
        CMD_FRAMEDATA_CUSTOM
    };

    enum GLXEventThreadCommand
    {
        CMD_GLXEVENTTHREAD_ADD_PIPE,
        CMD_GLXEVENTTHREAD_REMOVE_PIPE,
        CMD_GLXEVENTTHREAD_ADD_WINDOW,
        CMD_GLXEVENTTHREAD_REMOVE_WINDOW,
        CMD_GLXEVENTTHREAD_ALL
    };
};

#endif // EQ_COMMANDS_H

