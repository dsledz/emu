//
//  AppDelegate.m
//  DesktopEmu
//
//  Created by Dan Sledz on 8/7/13.
//  Copyright (c) 2013 Dan Sledz. All rights reserved.
//

#import "AppDelegate.h"

#include <OpenGL/gl.h>
#include "driver/wrapper.h"

@implementation AppDelegate
//-----------------
//NSOpenPanel: Displaying a File Open Dialog in OS X 10.7
//-----------------

// Any ole method
+ (NSString *)openDialog {

  // Loop counter.
  int i;

  // Create a File Open Dialog class.
  NSOpenPanel* openDlg = [NSOpenPanel openPanel];

  // Set array of file types
  NSMutableArray *fileTypesArray;
  fileTypesArray = [NSMutableArray arrayWithCapacity:0];

  /* XXX: This is excessive for a list of extensions. */
  struct emu *emu = emu_new();
  struct emu_machine *machines = emu_list_enum(emu);
  for (struct emu_machine *cur = machines; cur != NULL; cur = cur->next) {
      [fileTypesArray addObject:[NSString stringWithUTF8String:cur->extension]];
  }
  emu_list_free(machines);
  emu_free(emu);

  // Enable options in the dialog.
  [openDlg setCanChooseFiles:YES];
  [openDlg setAllowedFileTypes:fileTypesArray];
  [openDlg setAllowsMultipleSelection:FALSE];

  // Display the dialog box.  If the OK pressed,
  // process the files.
  if ( [openDlg runModal] == NSOKButton ) {

    // Gets list of all files selected
    NSArray *files = [openDlg URLs];

    return [NSString stringWithString:[[files objectAtIndex:0] path]];

  }

  return nil;
}

- (void)dealloc
{
    [super dealloc];
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    NSLog(@"App finished loading..");
}

@end

@implementation MyWindow

@end

@implementation MyWindowController
- (id) init {

    if ( ! (self = [super initWithWindowNibName: @"MainWindow"]) ) {
        NSLog(@"init failed in TheWindowController");
        return nil;
    }
    NSLog(@"init OK in TheWindowController");

    return self;
}

- (void)windowDidLoad {
    NSLog(@"TheWindowPanel did load");
}

@end
