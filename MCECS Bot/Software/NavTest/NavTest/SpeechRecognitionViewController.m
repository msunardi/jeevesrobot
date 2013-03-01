//
//  SpeechRecognitionViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/25/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "SpeechRecognitionViewController.h"
#import <OpenEars/LanguageModelGenerator.h>

LanguageModelGenerator *lmGenerator;
NSArray *words;
NSString *name;
NSError *err;
NSDictionary *languageGeneratorResults;
NSString *lmPath;
NSString *dicPath;

@interface SpeechRecognitionViewController ()

@end

@implementation SpeechRecognitionViewController

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

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setSwitchButton:nil];
    [self setStatusLabel:nil];
    [self setTextLabel:nil];
    [super viewDidUnload];
}
- (IBAction)switchFlipped:(id)sender {
    if (self.switchButton.on) {
        NSLog(@"Flipped on!");

        self.switchButton.enabled = NO; // Disable switch while pocketsphinx is loading
        [self.openEarsEventsObserver setDelegate:self];
        
        [self.pocketsphinxController startListeningWithLanguageModelAtPath:lmPath dictionaryAtPath:dicPath languageModelIsJSGF:NO];
    } else {
        NSLog(@"Flipped off!");
        [self.pocketsphinxController stopListening];
        [self.pocketsphinxController stopVoiceRecognitionThread];
        [self.openEarsEventsObserver setDelegate:nil];
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
    [self.statusLabel setText:@"I think you said:"];
    
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
    [self.statusLabel setText:@"Calibrating Pocketsphinx ..."];
}

- (void) pocketsphinxDidCompleteCalibration {
	NSLog(@"Pocketsphinx calibration is complete.");
    [self.statusLabel setText:@"Calibration complete!"];
}

- (void) pocketsphinxDidStartListening {
	NSLog(@"Pocketsphinx is now listening.");
    [self.statusLabel setText:@"Go ahead. I'm listening ..."];
    self.switchButton.enabled = YES;
}

- (void) pocketsphinxDidDetectSpeech {
	NSLog(@"Pocketsphinx has detected speech.");
    [self.statusLabel setText:@"Did somone say something?"];
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
