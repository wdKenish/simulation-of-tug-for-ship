def max_number(a,b,c):
    return max(a,b,c)


def is_prime(a):
    for i in range(2,int(a**0.5)+1):
        if a % i==0:
            return print(False)
            break
        else:
            if i == int(a**0.5)+1:
                return print(True)
            continue 
            
class Rectangle:
    def __init__(self,width,height):
        self.width=width
        self.height=height
    def area(self):
        return self.width * self.height
    def perimeter(self):
        return 2* (self.self + self.height)
        
class BankAccount:
    def __init__(self,fortune):
        self.fortune= fortune
    def deposit(self,amount):
        return self.fortune += amount
    def withdraw(self,amount):
        return self.fortune -= amount
    def get_balance(self):
        return self.fortune
        
class Vehicle:
    def __init__(self,make,model):
        self.make = make
        self.model = model
    def start_engine(self):
        return None

class Car(Vehicle):
    def __init__(self,make,model,num_doors):
        self.make = make. # mistake for inheritance 
        self.model = model
        self.num_doors = num_doors
    
class Truck(Vehicle):
    def __init__(self,make,model,load_capacity):
        super().__init__(make,model)
        self. capacity =capacity
        
      