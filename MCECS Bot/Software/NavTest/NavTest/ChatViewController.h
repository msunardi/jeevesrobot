//
//  ChatViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/14/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <OpenEars/PocketsphinxController.h>
#import <OpenEars/OpenEarsEventsObserver.h>

NSInputStream * inputStream;
NSOutputStream * outputStream;
NSMutableArray * messages;
NSInteger selectedIndex;


@protocol ChatViewControllerProtocol;
@class ChatViewController;

@protocol ChatViewControllerProtocol <NSObject>

- (void)setName:(NSString *)userName;
- (void)setServer:(NSString *)serverName;
- (void)cancelModal;

@end

#import "ChatModalViewController.h"


@interface ChatViewController : UIViewController <NSStreamDelegate, UITableViewDelegate, UITableViewDataSource,ChatModalDelegate, ChatViewControllerProtocol, UITextFieldDelegate, OpenEarsEventsObserverDelegate> {
    PocketsphinxController *pocketsphinxController;
    OpenEarsEventsObserver *openEarsEventsObserver;
}

- (IBAction)chatViewChangeUser:(id)sender;
- (IBAction)chatViewEnterText:(id)sender;

@property (weak, nonatomic) IBOutlet UIButton *chatViewEnterButton;
@property (weak, nonatomic) NSString *userName;
@property (weak, nonatomic) IBOutlet UITextField *chatViewInputText;
@property (weak, nonatomic) IBOutlet UITableView *chatTable;
@property (weak, nonatomic) NSString *chatServer;
- (IBAction)speechRecognitionSwitchFlip:(id)sender;
@property (weak, nonatomic) IBOutlet UISwitch *speechRecognitionSwitch;
@property (weak, nonatomic) IBOutlet UIView *speechRecognitionDiagnostics;

@property (strong,nonatomic) PocketsphinxController *pocketsphinxController;
@property (strong,nonatomic) OpenEarsEventsObserver *openEarsEventsObserver;
@property (weak, nonatomic) IBOutlet UILabel *speechStatus;
@property (weak, nonatomic) IBOutlet UILabel *textLabel;

@end




