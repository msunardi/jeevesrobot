//
//  Faculty.h
//  NavTest
//
//  Created by Mathias Sunardi on 2/24/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface Faculty : NSObject

@property (assign) int *faculty_id;
@property (nonatomic, strong) NSString *first_name;
@property (nonatomic, strong) NSString *middle_name;
@property (nonatomic, strong) NSString *last_name;
@property (nonatomic, strong) NSString *full_name;
@property (nonatomic, strong) NSString *department;
@property (nonatomic, strong) NSString *professorship;
@property (nonatomic, strong) NSString *research_area;
@property (nonatomic, strong) NSString *classes;
@property (nonatomic, strong) NSString *phone;
@property (nonatomic, strong) NSString *office;
@property (nonatomic, strong) NSString *office_hours;
@property (nonatomic, strong) NSString *email;
@property (nonatomic, strong) NSString *website;
@property (assign) int *labs;
@property (nonatomic, strong) NSString *other_roles;
@property (nonatomic, strong) NSString *image;
@property (nonatomic, strong) NSString *info;

@end
