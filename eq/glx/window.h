
/* Copyright (c) 2005-2017, Stefan Eilemann <eile@equalizergraphics.com>
 *                          Daniel Nachbaur <danielnachbaur@gmail.com>
 *                          Maxim Makhinya
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef EQ_GLX_WINDOW_H
#define EQ_GLX_WINDOW_H

#include <eq/glWindow.h> // base class
#include <eq/glx/types.h>

namespace eq
{
namespace glx
{
namespace detail
{
class Window;
}

/** The interface defining the minimum functionality for a glX window. */
class WindowIF : public GLWindow
{
public:
    WindowIF(NotifierInterface& parent, const WindowSettings& settings)
        : GLWindow(parent, settings)
    {
    }
    virtual ~WindowIF() {}
    /** @return the glX rendering context. @version 1.0 */
    virtual GLXContext getGLXContext() const = 0;

    /** @return the X11 drawable ID. @version 1.0 */
    virtual XID getXDrawable() const = 0;

    /** @return X11 display connection. @version 1.0 */
    virtual Display* getXDisplay() = 0;

    /** Process a (re)size event. @return true if the event was handled. */
    virtual bool processEvent(EventType type, const XEvent&, SizeEvent& event)
    {
        return GLWindow::processEvent(type, event);
    }

    /** Process a mouse event. @return true if the event was handled. */
    virtual bool processEvent(EventType type, const XEvent&,
                              PointerEvent& event)
    {
        return GLWindow::processEvent(type, event);
    }

    /** Process a keyboard event. @return true if the event was handled. */
    virtual bool processEvent(EventType type, const XEvent&, KeyEvent& event)
    {
        return GLWindow::processEvent(type, event);
    }

    /** Process an axis event. @return true if the event was handled. */
    virtual bool processEvent(const XEvent&, AxisEvent& event)
    {
        return GLWindow::processEvent(event);
    }

    /** Process a button event. @return true if the event was handled. */
    virtual bool processEvent(const XEvent&, ButtonEvent& event)
    {
        return GLWindow::processEvent(event);
    }

    /** Process a stateless event. @return true if the event was handled. */
    virtual bool processEvent(EventType type, const XEvent&)
    {
        if (type == EVENT_UNKNOWN)
            return false;
        return GLWindow::processEvent(type);
    }
};

/** Equalizer default implementation of a glX window */
class Window : public WindowIF
{
public:
    /**
     * Construct a new glX/X11 system window.
     * @version 1.7.2
     */
    Window(NotifierInterface& parent, const WindowSettings& settings,
           Display* xDisplay, const GLXEWContext* glxewContext,
           MessagePump* messagePump);

    /** Destruct this glX window. @version 1.0 */
    virtual ~Window();

    /** @name GLX/X11 initialization */
    //@{
    /**
     * Initialize this window for the glX window system.
     *
     * This method first call chooseGLXFBConfig(), then createGLXContext()
     * with the chosen framebuffer config, and finally creates a drawable
     * using configInitGLXDrawable().
     *
     * @return true if the initialization was successful, false otherwise.
     * @version 1.0
     */
    bool configInit() override;

    /** @version 1.0 */
    void configExit() override;

    /**
     * Choose a GLX framebuffer config based on the window's attributes.
     *
     * The returned FB config has to be freed using XFree().
     *
     * @return a pixel format, or 0 if no pixel format was found.
     * @version 1.0
     */
    virtual GLXFBConfig* chooseGLXFBConfig();

    /**
     * Create a glX context.
     *
     * This method does not set the window's glX context.
     *
     * @param fbConfig the framebuffer config for the context.
     * @return the context, or 0 if context creation failed.
     * @version 1.0
     */
    virtual GLXContext createGLXContext(GLXFBConfig* fbConfig);

    /**
     * Initialize the window's drawable and bind the glX context.
     *
     * Sets the window's X11 drawable on success
     *
     * @param fbConfig the framebuffer config for the context.
     * @return true if the drawable was created, false otherwise.
     * @version 1.0
     */
    virtual bool configInitGLXDrawable(GLXFBConfig* fbConfig);

    /**
     * Initialize the window with a window and bind the glX context.
     *
     * Sets the window's X11 drawable on success
     *
     * @param fbConfig the framebuffer config for the context.
     * @return true if the window was created, false otherwise.
     * @version 1.0
     */
    virtual bool configInitGLXWindow(GLXFBConfig* fbConfig);

    /**
     * Register with the pipe's GLXEventHandler, called by setXDrawable().
     * @version 1.0
     */
    virtual void initEventHandler();

    /**
     * Deregister with the GLXEventHandler, called by setXDrawable().
     * @version 1.0
     */
    virtual void exitEventHandler();
    //@}

    /** @name Data Access. */
    //@{
    /** @return the glX rendering context. @version 1.0 */
    GLXContext getGLXContext() const override;

    /**  @return  the X11 drawable ID. @version 1.0 */
    XID getXDrawable() const override;

    /** @return the X11 display. @version 1.0 */
    Display* getXDisplay() override;

    /** @return the GLXEW context. @version 1.0*/
    const GLXEWContext* glxewGetContext() const;

    /**
     * Set the X11 drawable ID for this window.
     *
     * This function should only be called from configInit() or
     * configExit().
     *
     * @param drawable the X11 drawable ID.
     * @version 1.0
     */
    virtual void setXDrawable(XID drawable);

    /**
     * Set the glX rendering context for this window.
     *
     * This function should only be called from configInit() or
     * configExit().
     * The context has to be set to 0 before it is destroyed.
     *
     * @param context the glX rendering context.
     * @version 1.0
     */
    virtual void setGLXContext(GLXContext context);
    //@}

    /** @name Operations. */
    //@{
    /** @version 1.0 */
    void makeCurrent(const bool cache = true) const override;

    /** @version 1.10 */
    void doneCurrent() const override;

    /** @version 1.0 */
    void swapBuffers() override;

    /** Implementation untested for glX. @version 1.0 */
    void joinNVSwapBarrier(const uint32_t group,
                           const uint32_t barrier) override;

    /** Unbind a GLX_NV_swap_barrier. @version 1.0 */
    void leaveNVSwapBarrier();

    bool processEvent(EventType type, const XEvent& xEvent,
                      PointerEvent& event) override;
    //@}

private:
    detail::Window* const _impl;

    /** Create an unmapped X11 window. */
    XID _createGLXWindow(GLXFBConfig* fbConfig, const PixelViewport& pvp);

    /** Init sync-to-vertical-retrace setting. */
    void _initSwapSync();

    void _resize(const PixelViewport& pvp) override;
};
}
}
#endif // EQ_GLX_WINDOW_H
