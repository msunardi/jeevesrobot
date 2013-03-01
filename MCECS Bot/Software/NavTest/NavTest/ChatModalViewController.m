//
//  ChatModalViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/14/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "ChatModalViewController.h"

@interface ChatModalViewController ()

@end

@implementation ChatModalViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    self.chatModalNameField.delegate = self;
    [self.chatModalNameField becomeFirstResponder];
    self.chatModalServerId.delegate = self;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {

    [self setChatModalNameField:nil];
    [self setChatModalServerId:nil];
    [super viewDidUnload];
}
- (IBAction)chatModalCancel:(id)sender {
    [self.delegate cancelModal];
    [self dismissModalViewControllerAnimated:YES];
}

- (IBAction)chatModalEnter:(id)sender {
    modalUserName = [[NSString alloc]init];
    modalUserName = self.chatModalNameField.text;
    
    NSLog(@"name: %@",self.chatModalNameField.text);
    
    [self modalSetServer:self.chatModalServerId.text];
    [self modalSetName:modalUserName];
    [self dismissModalViewControllerAnimated:YES];
}

- (BOOL)textFieldShouldReturn:(UITextField *)textField {
    NSString *userName = self.chatModalNameField.text;
    
    [self modalSetServer:self.chatModalServerId.text];
    [self modalSetName:userName];
    [self dismissModalViewControllerAnimated:YES];
    return YES;
}

- (void)modalSetName:(NSString *)userName {
    NSString *theUserName = [[NSString alloc]initWithFormat:@"%@",userName];
    [self.delegate setName:theUserName];
}

- (void)modalSetServer:(NSString *)serverName {
    serverName = [serverName stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
    NSString *theServerName = [[NSString alloc]initWithFormat:@"%@",serverName];
    if ([theServerName length]==0) {
        theServerName = [NSString stringWithFormat:@"localhost"];
    }
    [self.delegate setServer:theServerName];
}

@end
