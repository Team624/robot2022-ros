class Window:
    def __init__(self, maxLen=5):
        self.len = maxLen
        self.list = []
    
    def add(self, value):
        if len(self.list)==self.len:
            self.list = self.list[1:]
        self.list.append(value)
    
    def getAverage(self):
        if len(self.list)==0:
            return None
        else:
            return sum(self.list)/len(self.list)

    def getList(self):
        return self.list
    
    def getLen(self):
        return len(self.list)
    
