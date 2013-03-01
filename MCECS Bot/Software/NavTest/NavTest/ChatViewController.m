//
//  ChatViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/14/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#define COMMENT_LABEL_WIDTH 230
#define COMMENT_LABEL_MIN_HEIGHT 65
#define COMMENT_LABEL_PADDING 10

#import "ChatViewController.h"
#import "ChatModalViewController.h"
#import <OpenEars/LanguageModelGenerator.h>

LanguageModelGenerator *lmGenerator;
NSArray *words;
NSString *name;
NSError *err;
NSDictionary *languageGeneratorResults;
NSString *lmPath;
NSString *dicPath;


@interface ChatViewController ()

@end

@implementation ChatViewController

@synthesize pocketsphinxController;
@synthesize openEarsEventsObserver;

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
    //[self initNetworkCommunication];
    messages = [[NSMutableArray alloc]init];
    selectedIndex = -1;
    self.chatViewInputText.delegate = self;
    [self setChatServer:@"localhost"];
    
    @try {
        
        lmGenerator = [[LanguageModelGenerator alloc]init];
        words = [NSArray arrayWithObjects:@"HELLO", @"HELLO WORLD", @"COMPUTER", @"COFFEE", @"GOOD MORNING", @"ROBOTICS LAB", @"PORTLAND", @"I AM HAPPY", @"INTERESTING", @"I DO NOT KNOW", @"THE DEAN", @"RUNNING AROUND", @"ONE TWO THREE FOUR FIVE SIX SEVEN EIGHT NINE TEN", @"I BEG YOUR PARDON", nil];
        name = @"MyLanguageModelFile";
        err = [lmGenerator generateLanguageModelFromArray:words withFilesNamed:name];
        languageGeneratorResults = nil;
        lmPath = nil;
        dicPath = nil;
        
        if ([err code] == noErr) {
            languageGeneratorResults = [err userInfo];
            lmPath = [languageGeneratorResults objectForKey:@"LMPath"];
            dicPath = [languageGeneratorResults objectForKey:@"DictionaryPath"];
        } else {
            NSLog(@"Error: %@",[err localizedDescription]);
        }
    }
    @catch (NSException *exception) {
        NSLog(@"Error during load: %@",exception);
    }
}

//http://stackoverflow.com/questions/8221787/perform-segue-on-viewdidload
- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
    UIStoryboard *storyboard = [UIStoryboard storyboardWithName:@"Storyboard" bundle:nil];
    UIViewController *vc = [storyboard instantiateViewControllerWithIdentifier:@"ChatModalView"];
    [vc setModalPresentationStyle:UIModalPresentationFormSheet];
    ChatModalViewController *vcmodal = (ChatModalViewController *)vc;
    vcmodal.delegate = (id)self;
    
    // Connect self to server first
    /*NSString *myName = @"Jeeves";
    NSString *myMessage = [NSString stringWithFormat:@"iam:%@", myName];
    NSData *myData = [[NSData alloc] initWithData:[myMessage dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[myData bytes] maxLength:[myData length]];*/
    
    
    [self presentModalViewController:vcmodal animated:YES];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


- (void)viewDidUnload {
    [self setChatViewInputText:nil];
    [self setChatTable:nil];
    [self setChatViewEnterButton:nil];
    @try {
        [inputStream close];
        [inputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        [outputStream close];
        [outputStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        [inputStream setDelegate:nil];
        inputStream = nil;
        [outputStream setDelegate:nil];
        outputStream = nil;
        [messages removeAllObjects];
    }
    @catch (NSException *e) {
        NSLog(@"Error unloading view: %@",e.reason);
    }
    [self setSpeechRecognitionSwitch:nil];
    [self setSpeechRecognitionDiagnostics:nil];
    [self setSpeechStatus:nil];
    [self setTextLabel:nil];
    [super viewDidUnload];
}

- (IBAction)chatViewChangeUser:(id)sender {
    UIStoryboard *storyboard = [UIStoryboard storyboardWithName:@"Storyboard" bundle:nil];
    UIViewController *vc = [storyboard instantiateViewControllerWithIdentifier:@"ChatModalView"];
    [vc setModalPresentationStyle:UIModalPresentationFormSheet];
    ChatModalViewController *vcmodal = (ChatModalViewController *)vc;
    vcmodal.delegate = (id)self;
    [messages removeAllObjects];
    [self presentModalViewController:vcmodal animated:YES];
}

- (IBAction)chatViewEnterText:(id)sender {
    NSString *inputText = self.chatViewInputText.text;
    [self writeEnteredText:inputText];
    /*if ([inputText length] == 0) {
        return;
    }
    NSLog(@"text length: %d",[inputText length]);
    NSString *response = [NSString stringWithFormat:@"msg:%@", inputText];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
    self.chatViewInputText.text = @""; // clears the input field*/
}

- (BOOL)textFieldShouldReturn:(UITextField *)textField {
    NSString *inputText = self.chatViewInputText.text;
    
    [self writeEnteredText:inputText];
    /*NSString *response = [NSString stringWithFormat:@"msg:%@", inputText];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
    self.chatViewInputText.text = @""; // clears the input field*/
    return YES;
}

- (void)writeEnteredText:(NSString *)inputText {
    inputText = [inputText stringByTrimmingCharactersInSet:[NSCharacterSet whitespaceAndNewlineCharacterSet]];
    if ([inputText length] == 0) {
        return;
    }
    NSLog(@"text length: %d",[inputText length]);
    NSString *response = [NSString stringWithFormat:@"msg:%@", inputText];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
    self.chatViewInputText.text = @""; // clears the input field
}

/*- (IBAction)chatViewHints:(id)sender {
}*/

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
    UIViewController *controller = [segue destinationViewController];
    if ([segue.identifier isEqualToString:@"ChatModalView"]) {
        ChatModalViewController *chatModalController = (ChatModalViewController *)controller;
        chatModalController.delegate = (id)self;
        NSLog(@"WHAT AM I DOING???");
    }
}

- (void)setName:(NSString *)userName {
        
    // Then connect user to server
    self.userName = userName;
    self.navigationItem.title = userName;
    NSString *nameString = self.userName;
    NSString *response = [NSString stringWithFormat:@"iam:%@", nameString];
    NSData *data = [[NSData alloc] initWithData:[response dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
    
    // Then greet user
    //NSString *welcome = [NSString stringWithFormat:@"msg:Hello, %@",self.userName];
    //NSData *newdata = [[NSData alloc] initWithData:[welcome dataUsingEncoding:NSASCIIStringEncoding]];
    //[outputStream write:[newdata bytes] maxLength:[newdata length]];
    
    [self messageReceived:[NSString stringWithFormat:@"Jeeves: Hello, %@.", self.userName]];
    [self.chatViewEnterButton setEnabled:YES];
    [self.chatViewInputText setEnabled:YES];
    NSLog(@"setname from chatview");
}

- (void)setServer:(NSString *)serverName {
    NSLog(@"Server ID: %@",serverName);
    NSString *theServerName = [[NSString alloc]initWithFormat:@"%@",serverName];
    [self setChatServer:theServerName];
    [self initNetworkCommunication];
}

#pragma mark Modal methods
- (void)cancelModal {
    [self.chatViewInputText setEnabled:NO];
    [self.chatViewEnterButton setEnabled:NO];
    [messages removeAllObjects];
    [self.chatTable reloadData];
    NSLog(@"Cancel Modal");
}

- (void)chatModalView:(ChatModalViewController *)controller setName:(NSString *)username {
    NSLog(@"I got this from the modal view: %@", username);    
}
- (void)chatModalView:(ChatModalViewController *)controller server:(NSString *)serverId {
    NSLog(@"Server ID: %@",serverId);
}

- (void)chatModalViewCancelled:(ChatModalViewController *)controller {
    [self.chatViewInputText setEnabled:NO];
    [self.chatViewEnterButton setEnabled:NO];
}

// Table protocols
- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
    NSLog(@"messages count: %d",[messages count]);
    return [messages count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    static NSString *CellIdentifier = @"ChatCellIdentifier";
    
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:CellIdentifier];
    if (cell == nil) {
        cell = [[UITableViewCell alloc] initWithStyle:UITableViewCellStyleDefault reuseIdentifier:CellIdentifier];
        [cell setAutoresizesSubviews:YES];
    }
    
    /*UILabel* label = [[UILabel alloc] initWithFrame:CGRectMake(40, 0, tableView.frame.size.width, 70)];
    label.tag = 100;
    label.numberOfLines = 5;
    label.lineBreakMode = UILineBreakModeWordWrap;*/
    
    
    /* http://www.roostersoftstudios.com/2011/04/14/iphone-uitableview-with-animated-expanding-cells/ */
    //If this is the selected index then calculate the height of the cell based on the amount of text we have
    if(selectedIndex == indexPath.row)
    {
        
        CGFloat labelHeight = [self getLabelHeightForIndex:indexPath.row];
        cell.textLabel.frame = CGRectMake(cell.textLabel.frame.origin.x,
                                                 cell.textLabel.frame.origin.y,
                                                 cell.textLabel.frame.size.width,
                                                 labelHeight);
        
    }
    else {
        //Otherwise just return the minimum height for the label.
        cell.textLabel.frame = CGRectMake(cell.textLabel.frame.origin.x,
                                                 cell.textLabel.frame.origin.y,
                                                 cell.textLabel.frame.size.width,
                                                 COMMENT_LABEL_MIN_HEIGHT);
        
    }
    /* from http://www.roostersoftstudios.com/2011/04/14/iphone-uitableview-with-animated-expanding-cells/ */
    
    NSString *s = (NSString *) [messages objectAtIndex:indexPath.row];
    cell.textLabel.text = s;
    cell.textLabel.font = [UIFont systemFontOfSize:40];
    cell.textLabel.adjustsFontSizeToFitWidth = YES;
    //label.text = s;
    //[cell addSubview:label];

    cell.textLabel.numberOfLines = 3;
    
    return cell;
}


// Chat methods
- (void) messageReceived:(NSString *)message {
    [messages addObject:message];
    [self.chatTable reloadData];
    NSLog(@"message count: %d", messages.count);
    NSLog(@"message at index 0: %s", [[messages objectAtIndex:0] UTF8String]);
    NSIndexPath *topIndexPath = [NSIndexPath indexPathForRow:messages.count-1 inSection:0];
    [self.chatTable scrollToRowAtIndexPath:topIndexPath atScrollPosition:UITableViewScrollPositionMiddle animated:YES];
}

// Network stream methods
- (void)initNetworkCommunication {
    CFReadStreamRef readStream;
    CFWriteStreamRef writeStream;
    //CFStreamCreatePairWithSocketToHost(NULL, (CFStringRef)@"localhost", 80, &readStream, &writeStream);
    CFStreamCreatePairWithSocketToHost(NULL, (CFStringRef)CFBridgingRetain(self.chatServer), 80, &readStream, &writeStream);
    inputStream = (__bridge NSInputStream *)readStream;
    outputStream = (__bridge NSOutputStream *)writeStream;
    [inputStream setDelegate:self];
    [outputStream setDelegate:self];
    [inputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [inputStream open];
    [outputStream open];
}

- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)eventCode {
    NSLog(@"stream event %i", eventCode);
    
    switch (eventCode) {
        case NSStreamEventOpenCompleted:
            NSLog(@"Stream opened.");
            break;
            
        case NSStreamEventHasBytesAvailable:
            if (theStream == inputStream) {
                uint8_t buffer[1024];
                int len;
                
                while ([inputStream hasBytesAvailable]) {
                    len = [inputStream read:buffer maxLength:sizeof(buffer)];
                    if (len > 0) {
                        NSString *output = [[NSString alloc] initWithBytes:buffer length:len encoding:NSASCIIStringEncoding];
                        
                        if (nil != output) {
                            NSLog(@"Server said: %@", output);
                            [self messageReceived:output];
                        }
                    }
                }
            }
            break;
            
        case NSStreamEventErrorOccurred:
            NSLog(@"Cannot connect to the host!");
            break;
            
        case NSStreamEventEndEncountered:
            //[theStream close];
            //`[theStream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
            
        default:
            NSLog(@"Unknown event detected ...");
    }
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    [super touchesBegan:touches withEvent:event];
    [[self chatViewInputText]resignFirstResponder];
}

///// TO BE REMOVED
//This just a convenience function to get the height of the label based on the comment text
/* http://www.roostersoftstudios.com/2011/04/14/iphone-uitableview-with-animated-expanding-cells/ */
-(CGFloat)getLabelHeightForIndex:(NSInteger)index
{
    CGSize maximumSize = CGSizeMake(COMMENT_LABEL_WIDTH, 10000);
    
    CGSize labelHeighSize = [[messages objectAtIndex:index] sizeWithFont: [UIFont fontWithName:@"Helvetica" size:14.0f]
                                                       constrainedToSize:maximumSize
                                                           lineBreakMode:UILineBreakModeWordWrap];
    return labelHeighSize.height;
    
}

-(CGFloat)tableView:(UITableView *)tableView heightForRowAtIndexPath:(NSIndexPath *)indexPath {
    return 100;
}

/*
 -(CGFloat)tableView:(UITableView *)tableView heightForRowAtIndexPath:(NSIndexPath *)indexPath {
 //If this is the selected index we need to return the height of the cell
 //in relation to the label height otherwise we just return the minimum label height with padding
 if(selectedIndex == indexPath.row)
 {
 return [self getLabelHeightForIndex:indexPath.row] + COMMENT_LABEL_PADDING * 2;
 }
 else {
 return COMMENT_LABEL_MIN_HEIGHT + COMMENT_LABEL_PADDING * 2;
 }
 }
 
 -(NSIndexPath *)tableView:(UITableView *)tableView willSelectRowAtIndexPath:(NSIndexPath *)indexPath {
 //We only don't want to allow selection on any cells which cannot be expanded
 NSLog(@"YACCCC!!!");
 if([self getLabelHeightForIndex:indexPath.row] > COMMENT_LABEL_MIN_HEIGHT)
 {
 return indexPath;
 }
 else {
 return nil;
 }
 }
 
 -(void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
 {
 
 //The user is selecting the cell which is currently expanded
 //we want to minimize it back
 if(selectedIndex == indexPath.row)
 {
 selectedIndex = -1;
 [tableView reloadRowsAtIndexPaths:[NSArray arrayWithObject:indexPath] withRowAnimation:UITableViewRowAnimationFade];
 
 return;
 }
 
 //First we check if a cell is already expanded.
 //If it is we want to minimize make sure it is reloaded to minimize it back
 if(selectedIndex >= 0)
 {
 NSIndexPath *previousPath = [NSIndexPath indexPathForRow:selectedIndex inSection:0];
 selectedIndex = indexPath.row;
 [tableView reloadRowsAtIndexPaths:[NSArray arrayWithObject:previousPath] withRowAnimation:UITableViewRowAnimationFade];
 }
 
 //Finally set the selected index to the new selection and reload it to expand
 selectedIndex = indexPath.row;
 [tableView reloadRowsAtIndexPaths:[NSArray arrayWithObject:indexPath] withRowAnimation:UITableViewRowAnimationFade];
 }
 */

#pragma mark Speech Recognition methods
- (IBAction)speechRecognitionSwitchFlip:(id)sender {
    if (self.speechRecognitionSwitch.on) {
        NSLog(@"Flipped on!");
        
        self.speechRecognitionSwitch.enabled = NO; // Disable switch while pocketsphinx is loading
        [self.openEarsEventsObserver setDelegate:self];
        
        [self.pocketsphinxController startListeningWithLanguageModelAtPath:lmPath dictionaryAtPath:dicPath languageModelIsJSGF:NO];
    } else {
        NSLog(@"Flipped off!");
        [self.pocketsphinxController stopListening];
        [self.pocketsphinxController stopVoiceRecognitionThread];
        [self.openEarsEventsObserver setDelegate:nil];
        [self.speechStatus setText:@"Speech Recognition is off."];
        [self.textLabel setText:@"..."];
    }
}

#pragma mark OpenEars methods
- (PocketsphinxController *)pocketsphinxController {
    if (pocketsphinxController == nil) {
        pocketsphinxController = [[PocketsphinxController alloc]init];
    }
    return pocketsphinxController;
}

- (OpenEarsEventsObserver *)openEarsEventsObserver {
    if (openEarsEventsObserver == nil) {
        openEarsEventsObserver = [[OpenEarsEventsObserver alloc]init];
    }
    return openEarsEventsObserver;
}

- (void) pocketsphinxDidReceiveHypothesis:(NSString *)hypothesis recognitionScore:(NSString *)recognitionScore utteranceID:(NSString *)utteranceID {
	NSLog(@"The received hypothesis is %@ with a score of %@ and an ID of %@", hypothesis, recognitionScore, utteranceID);
    [self.speechStatus setText:@"I think you said:"];
    
    NSPredicate *predicate;
    predicate = [NSPredicate predicateWithFormat:@"self contains %@",hypothesis];
    
    if ([predicate evaluateWithObject:@"I AM HAPPY"]) {
        [self.textLabel setText:@"YOU DON'T SAY!"];
    }
    else if ([predicate evaluateWithObject:@"I BEG YOUR PARDON"]) {
        [self.textLabel setText:[NSString stringWithFormat:@"%@?",hypothesis]];
    }
    else {
        [self.textLabel setText:hypothesis];
    }
}

- (void) pocketsphinxDidStartCalibration {
	NSLog(@"Pocketsphinx calibration has started.");
    [self.speechStatus setText:@"Calibrating Pocketsphinx ..."];
}

- (void) pocketsphinxDidCompleteCalibration {
	NSLog(@"Pocketsphinx calibration is complete.");
    [self.speechStatus setText:@"Calibration complete!"];
}

- (void) pocketsphinxDidStartListening {
	NSLog(@"Pocketsphinx is now listening.");
    [self.speechStatus setText:@"Go ahead. I'm listening ..."];
    self.speechRecognitionSwitch.enabled = YES;
}

- (void) pocketsphinxDidDetectSpeech {
	NSLog(@"Pocketsphinx has detected speech.");
    [self.speechStatus setText:@"Did somone say something?"];
}

- (void) pocketsphinxDidDetectFinishedSpeech {
	NSLog(@"Pocketsphinx has detected a period of silence, concluding an utterance.");
}

- (void) pocketsphinxDidStopListening {
	NSLog(@"Pocketsphinx has stopped listening.");
}

- (void) pocketsphinxDidSuspendRecognition {
	NSLog(@"Pocketsphinx has suspended recognition.");
}

- (void) pocketsphinxDidResumeRecognition {
	NSLog(@"Pocketsphinx has resumed recognition.");
}

- (void) pocketsphinxDidChangeLanguageModelToFile:(NSString *)newLanguageModelPathAsString andDictionary:(NSString *)newDictionaryPathAsString {
	NSLog(@"Pocketsphinx is now using the following language model: \n%@ and the following dictionary: %@",newLanguageModelPathAsString,newDictionaryPathAsString);
}

- (void) pocketSphinxContinuousSetupDidFail { // This can let you know that something went wrong with the recognition loop startup. Turn on OPENEARSLOGGING to learn why.
	NSLog(@"Setting up the continuous recognition loop has failed for some reason, please turn on OpenEarsLogging to learn more.");
}

@end
