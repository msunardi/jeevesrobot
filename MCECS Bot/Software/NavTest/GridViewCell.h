//
//  GridViewCell.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/22/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "AQGridViewCell.h"

@interface GridViewCell : AQGridViewCell {
    UIImageView * iconView;
}

@property (nonatomic, retain) UIImageView *imageView;
@property (nonatomic, retain) UILabel *nameLabel;
//@property (nonatomic, retain) UILabel *researchLabel;
@property (nonatomic, retain) UIImage * icon;
@end
