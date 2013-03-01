//
//  EceTabPersonViewController.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/17/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "DTGridView.h"

@interface EceTabPersonViewController : UIViewController <DTGridViewDataSource, DTGridViewDelegate> {
    NSArray *someArray;
    NSArray *colours;
    //DTGridView *gridView;
}
@property (weak, nonatomic) IBOutlet DTGridView *gridView;



@end
