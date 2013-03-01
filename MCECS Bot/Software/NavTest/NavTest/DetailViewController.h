//
//  DetailViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/12/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Person.h"

@interface DetailViewController : UIViewController

@property(nonatomic)Person *somePerson;
@property (weak, nonatomic) IBOutlet UILabel *nameLabel;
@property (weak, nonatomic) IBOutlet UILabel *ageLabel;
@property(nonatomic) BOOL *yessir;
@property (weak, nonatomic) IBOutlet UILabel *yesSirLabel;

@end
