//
//  FacultyDetailViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/25/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Faculty.h"

@interface FacultyDetailViewController : UIViewController {
    //Faculty *faculty;
}
@property (nonatomic,strong) Faculty *someFaculty;

@property (weak, nonatomic) IBOutlet UIImageView *facultyImage;
@property (weak, nonatomic) IBOutlet UILabel *facultyName;
@property (weak, nonatomic) IBOutlet UILabel *departmentLabel;
@property (weak, nonatomic) IBOutlet UILabel *positionLabel;
@property (weak, nonatomic) IBOutlet UILabel *officeLabel;
@property (weak, nonatomic) IBOutlet UILabel *emailLabel;
@property (weak, nonatomic) IBOutlet UILabel *websiteLabel;
@property (weak, nonatomic) IBOutlet UILabel *researchLabel;
@property (weak, nonatomic) IBOutlet UITextView *infoTextView;
- (IBAction)takemeButton:(id)sender;
@end
