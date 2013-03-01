//
//  EceDetailViewController.m
//  NavTest
//
//  Created by Mathias Sunardi on 2/13/13.
//  Copyright (c) 2013 Mathias Sunardi. All rights reserved.
//

#import "EceDetailViewController.h"

@interface EceDetailViewController () {
    
}

@end

@implementation EceDetailViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
        self.eceLab = [[EceLabs alloc]init];
        
    }
    return self;
}

- (void)viewDidLoad
{
    
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    
    NSLog(@"detailview: %@", self.eceLab.name);
    self.eceLabDetailName.text = self.eceLab.name;
    self.eceLabDetailRoom.text = self.eceLab.room;
    self.eceLabDetailWebsite.text = self.eceLab.website;
    self.eceLabDetailDirector.text = self.eceLab.director;
    self.eceLabDetailDescription.text = self.eceLab.description;
    //self.eceLabImage.image = [UIImage imageNamed:@"psu_logo.jpg"];
    
    int numberOfPages = 9;
    int imageWidth = 768;
    int imageHeight = 500;
    int num = numberOfPages;
    
    for (int i=0; i<num; i++) {
        UIImageView *images = [[UIImageView alloc]initWithImage:[UIImage imageNamed:[NSString stringWithFormat:@"ece_%d.jpg",i]]];
        images.frame = CGRectMake((i-1)*imageWidth,0,imageWidth,imageHeight);
        
        [self.eceLabScrollView addSubview:images];
        NSLog(@"loading image: %d",i);
        
    }
    [self.eceLabScrollView setDelegate:self];
    [self.eceLabScrollView setContentSize:CGSizeMake(imageWidth*(num-1), imageHeight)];
    [self.eceLabScrollView setPagingEnabled:YES];
    [self.eceLabPageControl setNumberOfPages:9];
    [self.eceLabPageControl setCurrentPage:0];
    
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewDidUnload {
    [self setEceLabImage:nil];
    [self setEceLabDetailName:nil];
    [self setEceLabDetailWebsite:nil];
    [self setEceLabDetailRoom:nil];
    [self setEceLabDetailDirector:nil];
    [self setEceLabDetailDescription:nil];
    [self setEceLabScrollView:nil];
    [self setEceLabPageControl:nil];
    [super viewDidUnload];
}
- (IBAction)eceLabPageControl:(id)sender {
    int page = self.eceLabPageControl.currentPage;
    CGRect frame = self.eceLabScrollView.frame;
    frame.origin.x = frame.size.width*page;
    frame.origin.y = 0;
    
    [self.eceLabScrollView scrollRectToVisible:frame animated:YES];
}

- (void)scrollViewDidEndDecelerating:(UIScrollView *)scrollView {
    int page = scrollView.contentOffset.x/scrollView.frame.size.width;
    self.eceLabPageControl.currentPage = page;
}

@end
