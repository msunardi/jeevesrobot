//
//  GridViewCell.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/22/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "GridViewCell.h"
#import <QuartzCore/QuartzCore.h>

@implementation GridViewCell

- (id) initWithFrame:(CGRect)frame reuseIdentifier:(NSString *)reuseIdentifier {
    self = [super initWithFrame:frame reuseIdentifier:reuseIdentifier];
    
    if (self) {
        UIView *mainView = [[UIView alloc] initWithFrame:CGRectMake(0, 0, 240, 240)];
        [mainView setBackgroundColor:[UIColor clearColor]];
        
        self.backgroundColor = [UIColor colorWithWhite: 0.95 alpha: 0.0];
        self.contentView.backgroundColor = self.backgroundColor;


        //mainView.opaque = NO;
        
        //UIImageView *frameImageView = [[UIImageView alloc] initWithFrame:CGRectMake(9, 4, 142, 117)];
        //[frameImageView setImage:[UIImage imageNamed:@"tab-mask.png"]];
        
        //self.imageView = [[UIImageView alloc] initWithFrame:CGRectMake(13, 8, 135, 84)];
        //self.nameLabel = [[UILabel alloc] initWithFrame:CGRectMake(13, 92, 127, 21)];
        //self.researchLabel = [[UILabel alloc] initWithFrame:CGRectMake(13, 115, 127, 21)];
        
        [self.nameLabel setFont:[UIFont systemFontOfSize:14]];
        //[self.researchLabel setFont:[UIFont systemFontOfSize:14]];
        //[mainView addSubview:self.imageView];
        //[mainView addSubview:frameImageView];
        //[mainView addSubview:self.nameLabel];
        //[mainView addSubview:self.researchLabel];
        //[self.contentView addSubview:mainView];
        UIBezierPath * path = [UIBezierPath bezierPathWithRoundedRect: CGRectMake(10.0, 10.0, 220.0, 220.0)
                                                         cornerRadius: 18.0];
        
        iconView = [[UIImageView alloc] initWithFrame: CGRectMake(20.0, 20.0, 200.0, 200.0)];
        iconView.backgroundColor = [UIColor clearColor];
        iconView.opaque = NO;
        iconView.layer.shadowPath = path.CGPath;
        iconView.layer.shadowRadius = 8.0;
        iconView.layer.shadowOpacity = 0.4;
        iconView.layer.shadowOffset = CGSizeMake( 5.0, 5.0 );
        
        [mainView addSubview: iconView];
        
        self.nameLabel = [[UILabel alloc] initWithFrame:CGRectMake(20, 168, 200, 52)];
        self.nameLabel.backgroundColor = [UIColor colorWithRed:0 green:0 blue:0 alpha:0.65];
        self.nameLabel.textColor = [UIColor whiteColor];
        self.nameLabel.textAlignment = NSTextAlignmentCenter;
        self.nameLabel.numberOfLines = 0;
        self.nameLabel.lineBreakMode = NSLineBreakByWordWrapping;
        [mainView addSubview:self.nameLabel];
        [self.contentView addSubview:mainView];
        
        self.contentView.backgroundColor = [UIColor clearColor];
        self.backgroundColor = [UIColor clearColor];
        
        self.contentView.opaque = NO;
        self.opaque = NO;
        
        self.selectionStyle = AQGridViewCellSelectionStyleGlow;
    }
    return self;
}

- (UIImage *) icon
{
    return ( iconView.image );
}

- (void) setIcon: (UIImage *) anIcon
{
    iconView.image = anIcon;
}

@end
