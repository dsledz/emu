#import <Cocoa/Cocoa.h>
#import <QuartzCore/CVDisplayLink.h>

@interface DisplayView : NSOpenGLView {
  CVDisplayLinkRef displayLink;
}

- (IBAction)doOpen:(id)pId;
- (IBAction)doResume:(id)pId;
- (IBAction)doPause:(id)pId;
- (IBAction)doStop:(id)pId;
- (IBAction)doReset:(id)pId;

@end

@interface DebugView : NSView {
}

@end

@interface CPUView : NSView {
}

@end

@interface CPURegisterField : NSForm {
}

@end
