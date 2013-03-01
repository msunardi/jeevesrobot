//
//  EceLabs.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface EceLabs : NSObject

@property(assign)int *labId;
@property(nonatomic, strong)NSString *name;
@property(nonatomic, strong)NSString *room;
@property(nonatomic, strong)NSString *director;
@property(nonatomic, strong)NSString *website;
@property(nonatomic, strong)NSString *description;

@end
