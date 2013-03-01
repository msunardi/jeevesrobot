//
//  NavTestBrowseViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/12/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "sqlite3.h"
#import "Person.h"
#import "DetailViewController.h"

@interface NavTestBrowseViewController : UIViewController <UITableViewDelegate, UITableViewDataSource>
@property (weak, nonatomic) IBOutlet UITextField *nameField;
@property (weak, nonatomic) IBOutlet UITextField *ageField;
@property (weak, nonatomic) IBOutlet UITableView *myTableView;

- (IBAction)addButton:(id)sender;
- (IBAction)showAllButton:(id)sender;
- (IBAction)deleteButton:(id)sender;
- (IBAction)nextButton:(id)sender;

@end
