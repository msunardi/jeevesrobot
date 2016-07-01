from collections import defaultdict

class Poses:
    def __init__(self, page):

        self.poses = page.find('Poses')
        self.poseclasses = [pc.findall('.//PoseClass') for pc in self.poses]
        # self.poses = page.find('Poses')
        self.title = page.find('Title').text
        self.body = {'R_SHO_PITCH': None,
                    'L_SHO_PITCH': None,
                    'R_SHO_ROLL': None,
                    'L_SHO_ROLL': None,
                    'R_ELBOW': None,
                    'L_ELBOW': None,
                    'R_HIP_YAW': None,
                    'L_HIP_YAW': None,
                    'R_HIP_ROLL': None,
                    'L_HIP_ROLL': None,
                    'R_HIP_PITCH': None,
                    'L_HIP_PITCH': None,
                    'R_KNEE': None,
                    'L_KNEE': None,
                    'R_ANK_PITCH': None,
                    'L_ANK_PITCH': None,
                    'R_ANK_ROLL': None,
                    'L_ANK_ROLL': None,
                    'HEAD_PAN': None,
                    'HEAD_TILT': None
                     }
        self.timing = {'Time': None,
                    'PauseTime': None
                    }
        # self.allpages = []
        
        self.extractPoses()
        
    def extractPoses(self):
#         self.pose_classes = [pc for pc in self.poses]
#         print self.poses[0]
#         print self.pose_classes
#         for pose_class in self.poses:
#             print pose_class
    
        # fbody = defaultdict(list)
        for key in self.body.keys():
            self.body[key] = self._getJointPoses(key, self.poses)

        for key in self.timing.keys():
            self.timing[key] = self._getJointPoses(key, self.poses)
        
        # return allpages
    
    def _getJointPoses(self, joint_name, poses):
        return [int(pcx.find(joint_name).text) for pcx in poses]
    
    def getTitle(self):
        return self.title
    
    def getPoses(self):
        return self.body

    def getTiming(self):
        return self.timing
