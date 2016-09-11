//
//  AppDelegate.h
//  DesktopEmu
//
//  Created by Dan Sledz on 8/7/13.
//  Copyright (c) 2013 Dan Sledz. All rights reserved.
//

#import <Cocoa/Cocoa.h>

struct emu_state;

@interface MyWindowController : NSWindowController {
}

@end

@interface AppDelegate : NSObject<NSApplicationDelegate> {
  MyWindowController *controller;
}

@property(assign) IBOutlet NSWindow *window;

+ (NSString *)openDialog;

@end

@interface MyWindow : NSWindow {
}

@property(assign) IBOutlet NSString *rom;

@end
