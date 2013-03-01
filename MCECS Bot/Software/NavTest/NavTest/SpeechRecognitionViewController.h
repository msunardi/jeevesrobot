//
//  SpeechRecognitionViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/25/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <OpenEars/PocketsphinxController.h>
#import <OpenEars/OpenEarsEventsObserver.h>

@interface SpeechRecognitionViewController : UIViewController <OpenEarsEventsObserverDelegate> {
    PocketsphinxController *pocketsphinxController;
    OpenEarsEventsObserver *openEarsEventsObserver;
}

@property (strong,nonatomic) PocketsphinxController *pocketsphinxController;
@property (strong,nonatomic) OpenEarsEventsObserver *openEarsEventsObserver;
@property (weak, nonatomic) IBOutlet UISwitch *switchButton;
@property (weak, nonatomic) IBOutlet UILabel *statusLabel;
@property (weak, nonatomic) IBOutlet UILabel *textLabel;
- (IBAction)switchFlipped:(id)sender;

@end
