from belt_geometry import Belt, BeltProfile


# The SharedVars class is needed to ensure nothing is lost in reassignments and garbage 
# collection.
class SharedVars:
    def __init__(self):
        self.raw_center_points = [None]
        self.valid_midline_belt: Belt = None
        self.preview_graphics = None
        self.profile: BeltProfile = None
        self.rowNumber = 0

