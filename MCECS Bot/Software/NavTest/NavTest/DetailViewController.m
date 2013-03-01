//
//  DetailViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/12/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "DetailViewController.h"

@interface DetailViewController ()

@end

@implementation DetailViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
        self.somePerson = [[Person alloc]init];
    }
    
    return self;
}


- (void)viewDidLoad
{
    
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    self.nameLabel.text = self.somePerson.name;
    self.ageLabel.text = [NSString stringWithFormat:@"%d", self.somePerson.age];
    if (self.yessir) {
        self.yesSirLabel.text = @"YES!";
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setNameLabel:nil];
    [self setAgeLabel:nil];
    [self setYesSirLabel:nil];
    [super viewDidUnload];
}
@end
