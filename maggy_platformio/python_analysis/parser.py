



def parse(filename):
    with open (filename, 'r') as f:
        lines = f.readlines()
        data = dict()
        for line in lines:
            line = line.strip()
            if line.startswith('t:'):
                # Parse key:value pairs
                parts = line.split(',')
                for part in parts:
                    if ':' in part:
                        key, val = part.split(':')
                        key = key.strip()
                        val = val.strip()
                        if not key in data:
                            data[key] = []
                            
                        data[key].append(float(val))
        return data