//
//  FacultyDetailViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/25/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "FacultyDetailViewController.h"

@interface FacultyDetailViewController ()

@end

@implementation FacultyDetailViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
        //self.someFaculty = [[Faculty alloc] init];
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    NSLog(@"Faculty detail view");
    self.facultyName.numberOfLines = 0;
    self.facultyName.lineBreakMode = NSLineBreakByWordWrapping;
    self.facultyName.text = self.someFaculty.full_name;
    self.departmentLabel.text = self.someFaculty.department;
    
    self.positionLabel.numberOfLines = 0;
    self.positionLabel.lineBreakMode = NSLineBreakByWordWrapping;
    self.positionLabel.text = self.someFaculty.professorship;
    NSLog(@"Professorship: %@", self.someFaculty.professorship);
    
    self.officeLabel.text = self.someFaculty.office;
    self.emailLabel.text = self.someFaculty.email;
    self.websiteLabel.text = self.someFaculty.website;
    self.researchLabel.numberOfLines = 0;
    self.researchLabel.lineBreakMode = NSLineBreakByWordWrapping;

    self.researchLabel.text = self.someFaculty.research_area;
    self.infoTextView.text = self.someFaculty.info;
    [self.facultyImage setImage:[UIImage imageNamed:self.someFaculty.image]];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setFacultyImage:nil];
    [self setFacultyName:nil];
    [self setDepartmentLabel:nil];
    [self setPositionLabel:nil];
    [self setOfficeLabel:nil];
    [self setEmailLabel:nil];
    [self setWebsiteLabel:nil];
    [self setResearchLabel:nil];
    [self setInfoTextView:nil];
    [self setSomeFaculty:nil];
    [super viewDidUnload];
}
- (IBAction)takemeButton:(id)sender {
}
@end
