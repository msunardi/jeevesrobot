//
//  ChatModalViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/14/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>


@class ChatModalViewController;
@protocol ChatModalDelegate <NSObject>

- (void)chatModalView:(ChatModalViewController *)controller setName:(NSString *)username;
- (void)chatModalViewCancelled:(ChatModalViewController *)controller;
- (void)chatModalView:(ChatModalViewController *)controller server:(NSString *)serverId;

@end
#import "ChatViewController.h"

@interface ChatModalViewController : UIViewController <UITextFieldDelegate>{
    NSString *modalUserName;
}

- (IBAction)chatModalCancel:(id)sender;
- (IBAction)chatModalEnter:(id)sender;
//- (IBAction)setName:(NSString *)userName:(id)sender;
@property (weak, nonatomic) IBOutlet UITextField *chatModalNameField;
@property (retain, nonatomic) id<ChatViewControllerProtocol> delegate;
@property (weak, nonatomic) id<ChatModalDelegate> delegatex;
@property (weak, nonatomic) IBOutlet UITextField *chatModalServerId;

@end
