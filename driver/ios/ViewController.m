//
//  ViewController.m
//  Emulator
//
//  Created by Dan Sledz on 7/23/13.
//  Copyright (c) 2013 Dan Sledz. All rights reserved.
//

#import "ViewController.h"

#include "driver/wrapper.h"

@interface ViewController () {
    GLKMatrix4 _modelViewProjectionMatrix;
    
    struct emu *_emu;
}
@property (strong, nonatomic) EAGLContext *context;
@property (strong, nonatomic) GLKBaseEffect *effect;

- (void)setupEMU;
- (void)tearDownEMU;

- (enum emu_key) map_key:(CGPoint)location;

@end

@implementation ViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    self.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];

    if (!self.context) {
        NSLog(@"Failed to create ES context");
    }
    
    GLKView *view = (GLKView *)self.view;
    view.context = self.context;
    view.drawableDepthFormat = GLKViewDrawableDepthFormat24;
    
    [self setupEMU];
}

- (void)dealloc
{    
    [self tearDownEMU];
    
    //[self tearDownGL];
    
    if ([EAGLContext currentContext] == self.context) {
        [EAGLContext setCurrentContext:nil];
    }
    [super dealloc];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];

    if ([self isViewLoaded] && ([[self view] window] == nil)) {
        self.view = nil;
        
        if ([EAGLContext currentContext] == self.context) {
            [EAGLContext setCurrentContext:nil];
        }
        self.context = nil;
    }

    // Dispose of any resources that can be recreated.
}

- (void)setupEMU
{
    [EAGLContext setCurrentContext:self.context];
    
    NSBundle *mainBundle = [NSBundle mainBundle];
#if 1
    NSString *myRom = [mainBundle pathForResource: @"mario" ofType:@"nes"];
    const char *driver = "nes";
#else
    NSString *myRom = [mainBundle pathForResource: @"drmario" ofType:@"gb"];
    const char *driver = "gb";
#endif
    const char *rom = [myRom cStringUsingEncoding:[NSString defaultCStringEncoding]];
    _emu = emu_new();

    emu_machine_load(_emu, driver, rom);
    emu_machine_start(_emu);
}

- (void)tearDownEMU
{
    [EAGLContext setCurrentContext:self.context];

    emu_machine_stop(_emu);

    emu_free(_emu);
}
#pragma mark - Touch

- (enum emu_key)map_key:(CGPoint)location
{
    if (self.view.bounds.size.height / 2 < location.y) {
        /* Split the screen into four regions */
        return EMU_KEY_NONE;
    } else {
        /* Select and start */
        if (self.view.bounds.size.width / 2 < location.x) {
            return EMU_KEY_JOY1SELECT;
        } else {
            return EMU_KEY_JOY1START;
        }
    }
    return EMU_KEY_NONE;
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    for (UITouch *touch in touches) {
        CGPoint location = [touch locationInView:self.view];

        struct emu_event event = {
            .key = [self map_key:location],
            .pressed = true,
        };
        emu_send_event(_emu, &event);
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    for (UITouch *touch in touches) {
        CGPoint location = [touch locationInView:self.view];
        NSLog(@"point: (%f, %f)", location.x, location.y);

        struct emu_event event = {
            .key = [self map_key:location],
            .pressed = false,
        };
        emu_send_event(_emu, &event);
    }
}

#pragma mark - GLKView and GLKViewController delegate methods

- (void)update
{
    // XXX: Not sure where this should go.
    float aspect = fabsf(self.view.bounds.size.width / self.view.bounds.size.height);
    GLKMatrix4 projectionMatrix = GLKMatrix4MakePerspective(GLKMathDegreesToRadians(65.0f), aspect, 0.1f, 100.0f);
    
    // Compute the model view matrix for the object rendered with ES2
    GLKMatrix4 baseModelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, -4.0f);
    GLKMatrix4 modelViewMatrix = GLKMatrix4MakeTranslation(0.0f, 0.0f, 1.5f);
    modelViewMatrix = GLKMatrix4Multiply(baseModelViewMatrix, modelViewMatrix);
    _modelViewProjectionMatrix = GLKMatrix4Multiply(projectionMatrix, modelViewMatrix);
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    glClearColor(0.65f, 0.65f, 0.65f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    emu_machine_render(_emu);
    
    glFlush();
}
@end
