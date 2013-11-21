/* XXX: Taken from Apple Example */

#import "DisplayView.h"
#import "AppDelegate.h"

#include "driver/wrapper.h"

#define SUPPORT_RETINA_RESOLUTION 1

@interface DisplayView (PrivateMethods)
- (void) initGL;

- (enum emu_key)interpretKey:(NSEvent *)theEvent;
@end

@implementation DisplayView

- (CVReturn) getFrameForTime:(const CVTimeStamp*)outputTime
{
    // There is no autorelease pool when this method is called
    // because it will be called from a background thread.
    // It's important to create one or app can leak objects.
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];

    [self drawView];

    [pool release];
    return kCVReturnSuccess;
}

// This is the renderer output callback function
static CVReturn MyDisplayLinkCallback(CVDisplayLinkRef displayLink,
                                      const CVTimeStamp* now,
                                      const CVTimeStamp* outputTime,
                                      CVOptionFlags flagsIn,
                                      CVOptionFlags* flagsOut,
                                      void* displayLinkContext)
{
    CVReturn result = [(DisplayView*)displayLinkContext getFrameForTime:outputTime];
    return result;
}

- (void) awakeFromNib
{
    emu = emu_new();

    NSOpenGLPixelFormatAttribute attrs[] =
    {
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFADepthSize, 24,
        // Must specify the 3.2 Core Profile to use OpenGL 3.2
#if ESSENTIAL_GL_PRACTICES_SUPPORT_GL3 
        NSOpenGLPFAOpenGLProfile,
        NSOpenGLProfileVersion3_2Core,
#endif
        0
    };

    NSOpenGLPixelFormat *pf = [[[NSOpenGLPixelFormat alloc] initWithAttributes:attrs] autorelease];

    if (!pf)
    {
        NSLog(@"No OpenGL pixel format");
    }

    NSOpenGLContext* context = [[[NSOpenGLContext alloc] initWithFormat:pf shareContext:nil] autorelease];

#if ESSENTIAL_GL_PRACTICES_SUPPORT_GL3 && defined(DEBUG)
    // When we're using a CoreProfile context, crash if we call a legacy OpenGL function
    // This will make it much more obvious where and when such a function call is made so
    // that we can remove such calls.
    // Without this we'd simply get GL_INVALID_OPERATION error for calling legacy functions
    // but it would be more difficult to see where that function was called.
    CGLEnable([context CGLContextObj], kCGLCECrashOnRemovedFunctions);
#endif

    [self setPixelFormat:pf];

    [self setOpenGLContext:context];

#if SUPPORT_RETINA_RESOLUTION
    // Opt-In to Retina resolution
    [self setWantsBestResolutionOpenGLSurface:YES];
#endif // SUPPORT_RETINA_RESOLUTION
}
 
- (void) prepareOpenGL
{
    [super prepareOpenGL];

    // Make all the OpenGL calls to setup rendering  
    //  and build the necessary rendering objects
    [self initGL];

    // Create a display link capable of being used with all active displays
    CVDisplayLinkCreateWithActiveCGDisplays(&displayLink);

    // Set the renderer output callback function
    CVDisplayLinkSetOutputCallback(displayLink, &MyDisplayLinkCallback, self);

    // Set the display link for the current renderer
    CGLContextObj cglContext = [[self openGLContext] CGLContextObj];
    CGLPixelFormatObj cglPixelFormat = [[self pixelFormat] CGLPixelFormatObj];
    CVDisplayLinkSetCurrentCGDisplayFromOpenGLContext(displayLink, cglContext, cglPixelFormat);

    // Activate the display link
    CVDisplayLinkStart(displayLink);

    // Register to be notified when the window closes so we can stop the displaylink
    [[NSNotificationCenter defaultCenter]
        addObserver:self
        selector:@selector(windowWillClose:)
        name:NSWindowWillCloseNotification
        object:[self window]];
}

- (void) windowWillClose:(NSNotification*)notification
{
    // Stop the display link when the window is closing because default
    // OpenGL render buffers will be destroyed.  If display link continues to
    // fire without renderbuffers, OpenGL draw calls will set errors.

    CVDisplayLinkStop(displayLink);
}

- (void) initGL
{
    // The reshape function may have changed the thread to which our OpenGL
    // context is attached before prepareOpenGL and initGL are called.  So call
    // makeCurrentContext to ensure that our OpenGL context current to this 
    // thread (i.e. makeCurrentContext directs all OpenGL calls on this thread
    // to [self openGLContext])
    [[self openGLContext] makeCurrentContext];

    // Synchronize buffer swaps with vertical refresh rate
    GLint swapInt = 1;
    [[self openGLContext]
        setValues:&swapInt
        forParameter:NSOpenGLCPSwapInterval];
}

- (void) reshape
{
    [super reshape];

    // We draw on a secondary thread through the display link. However, when
    // resizing the view, -drawRect is called on the main thread.
    // Add a mutex around to avoid the threads accessing the context
    // simultaneously when resizing.
    CGLLockContext([[self openGLContext] CGLContextObj]);

    // Get the view size in Points
    NSRect viewRectPoints = [self bounds];

#if SUPPORT_RETINA_RESOLUTION

    // Rendering at retina resolutions will reduce aliasing, but at the potential
    // cost of framerate and battery life due to the GPU needing to render more
    // pixels.

    // Any calculations the renderer does which use pixel dimentions, must be
    // in "retina" space.  [NSView convertRectToBacking] converts point sizes
    // to pixel sizes.  Thus the renderer gets the size in pixels, not points,
    // so that it can set it's viewport and perform and other pixel based
    // calculations appropriately.
    // viewRectPixels will be larger (2x) than viewRectPoints for retina displays.
    // viewRectPixels will be the same as viewRectPoints for non-retina displays
    NSRect viewRectPixels = [self convertRectToBacking:viewRectPoints];

#else //if !SUPPORT_RETINA_RESOLUTION

    // App will typically render faster and use less power rendering at
    // non-retina resolutions since the GPU needs to render less pixels.  There
    // is the cost of more aliasing, but it will be no-worse than on a Mac
    // without a retina display.

    // Points:Pixels is always 1:1 when not supporting retina resolutions
    NSRect viewRectPixels = viewRectPoints;

#endif // !SUPPORT_RETINA_RESOLUTION

    CGLUnlockContext([[self openGLContext] CGLContextObj]);
}


- (void)renewGState
{
    // Called whenever graphics state updated (such as window resize)

    // OpenGL rendering is not synchronous with other rendering on the OSX.
    // Therefore, call disableScreenUpdatesUntilFlush so the window server
    // doesn't render non-OpenGL content in the window asynchronously from
    // OpenGL content, which could cause flickering.  (non-OpenGL content
    // includes the title bar and drawing done by the app with other APIs)
    [[self window] disableScreenUpdatesUntilFlush];

    [super renewGState];
}

- (void) drawRect: (NSRect) theRect
{
    // Called during resize operations

    // Avoid flickering during resize by drawing
    [self drawView];
}

- (void) drawView
{
    [[self openGLContext] makeCurrentContext];

    // We draw on a secondary thread through the display link
    // When resizing the view, -reshape is called automatically on the main
    // thread. Add a mutex around to avoid the threads accessing the context
    // simultaneously when resizing
    CGLLockContext([[self openGLContext] CGLContextObj]);

    //glClearColor(0.0, 0.0, 0.0, 0.0);
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    emu_machine_render(emu);
    //glFlush();

    CGLFlushDrawable([[self openGLContext] CGLContextObj]);
    CGLUnlockContext([[self openGLContext] CGLContextObj]);
}

- (void) dealloc
{
    // Stop the display link BEFORE releasing anything in the view
    // otherwise the display link thread may call into the view and crash
    // when it encounters something that has been release
    CVDisplayLinkStop(displayLink);

    CVDisplayLinkRelease(displayLink);

    emu_machine_stop(emu);

    emu_free(emu);

    [super dealloc];
}

- (IBAction)doOpen:(id)pId
{
    NSString *rom = [AppDelegate openDialog];
    if (rom == nil)
        return;

    NSString *extension = [rom pathExtension];
    const char *rom_cstr = [rom cStringUsingEncoding:[NSString defaultCStringEncoding]];

    NSOpenGLContext* context = [self openGLContext];

    if ([context view] != self) {
        [context setView:self];
    }
    [context makeCurrentContext];

    emu_machine_stop(emu);
    struct emu_machine *machines = emu_list_enum(emu);
    struct emu_machine *machine = NULL;
    for (struct emu_machine *cur = machines; cur != NULL && machine == NULL;
         cur = cur->next) {
        NSString *ext = [NSString stringWithUTF8String:cur->extension];
        if ([extension isEqualToString:ext]) {
            machine = cur;
        }
    }

    if (machine != NULL) {
        emu_machine_load(emu, machine->driver, rom_cstr);
        emu_machine_start(emu);
    }

    emu_list_free(machines);
}

- (IBAction)doResume:(id)pId
{
    emu_machine_start(emu);
}

- (IBAction)doPause:(id)pId
{
    emu_machine_pause(emu);
}

- (IBAction)doStop:(id)pId
{
    emu_machine_stop(emu);
}

- (IBAction)doReset:(id)pId
{
    emu_machine_reset(emu);
}

- (BOOL) acceptsFirstResponder
{
    return TRUE;
}

- (enum emu_key)interpretKey:(NSEvent *)theEvent
{
    NSString *theKey = [theEvent charactersIgnoringModifiers];
    if ( [theKey length] == 0 )
        return EMU_KEY_NONE;
    unichar keyChar = [theKey characterAtIndex:0];
    switch (keyChar) {
    case NSLeftArrowFunctionKey:
        return EMU_KEY_JOY1LEFT;
    case NSRightArrowFunctionKey:
        return EMU_KEY_JOY1RIGHT;
    case NSUpArrowFunctionKey:
        return EMU_KEY_JOY1UP;
    case NSDownArrowFunctionKey:
        return EMU_KEY_JOY1DOWN;
    case 'z':
    case 'Z':
        return EMU_KEY_JOY1BTN1;
    case 'x':
    case 'X':
        return EMU_KEY_JOY1BTN2;
    case '\n':
    case '1':
        return EMU_KEY_JOY1START;
    case '\\':
    case 's':
        return EMU_KEY_JOY1SELECT;
    default:
        return EMU_KEY_NONE;
    }
}

- (void)keyDown:(NSEvent *)theEvent
{
    struct emu_event event = {
        .key = [self interpretKey:theEvent],
        .pressed = true,
    };
    emu_send_event(emu, &event);
}

- (void)keyUp:(NSEvent *)theEvent
{
    struct emu_event event = {
        .key = [self interpretKey:theEvent],
        .pressed = false,
    };
    emu_send_event(emu, &event);
}

@end
