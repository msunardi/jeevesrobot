//
//  EceDatabaseViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "sqlite3.h"
#import "EceLabs.h"
#import "SampleTableCell.h"
#import "EceDetailViewController.h"

@interface EceDatabaseViewController : UIViewController <UITableViewDelegate, UITableViewDataSource>
/*{
    EceDetailViewController *eceDetailView;
    UINavigationController *navController;
    UIStoryboard *storyBoard;
}*/

@property (nonatomic,retain) EceDetailViewController *eceDetailView;
@property (nonatomic,retain) UINavigationController *navController;
@property (nonatomic,retain) UIStoryboard *storyBoard;

@property (weak, nonatomic) IBOutlet UITableView *eceTableView;
@property (nonatomic,strong) EceLabs *someLab;

@end
