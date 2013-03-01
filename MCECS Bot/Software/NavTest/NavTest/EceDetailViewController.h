//
//  EceDetailViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "EceLabs.h"

@interface EceDetailViewController : UIViewController <UIScrollViewDelegate>

@property (nonatomic, strong)EceLabs *eceLab;
@property (weak, nonatomic) IBOutlet UIImageView *eceLabImage;
@property (weak, nonatomic) IBOutlet UILabel *eceLabDetailName;
@property (weak, nonatomic) IBOutlet UILabel *eceLabDetailWebsite;
@property (weak, nonatomic) IBOutlet UILabel *eceLabDetailRoom;
@property (weak, nonatomic) IBOutlet UILabel *eceLabDetailDirector;
@property (weak, nonatomic) IBOutlet UILabel *eceLabDetailDescription;
@property (weak, nonatomic) IBOutlet UIScrollView *eceLabScrollView;
@property (weak, nonatomic) IBOutlet UIPageControl *eceLabPageControl;
- (IBAction)eceLabPageControl:(id)sender;

@end
