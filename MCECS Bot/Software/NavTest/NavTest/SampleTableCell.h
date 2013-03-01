//
//  SampleTableCell.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface SampleTableCell : UITableViewCell
@property (weak, nonatomic) IBOutlet UILabel *labCellName;
@property (weak, nonatomic) IBOutlet UILabel *labCellRoom;
@property (weak, nonatomic) IBOutlet UILabel *labCellDescription;
@property (weak, nonatomic) IBOutlet UIImageView *labCellImage;

@end
