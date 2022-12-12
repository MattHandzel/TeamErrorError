import math
import time

def get_factors(n):
  num = 2
  sqrt_n = math.sqrt(n)
  factors = [n]
  while True:
    if n % num == 0:
      factors.append(num)
      factors.append(int(n / num))
    if num > sqrt_n:
      return factors
    
    num += 1

def ecg_seq_index(n):
  seq = [1, 2]  
  d_t = []
  
  for i in range(n):
    s_t = time.perf_counter()
    factors = get_factors(seq[-1])
    results = []
    for factor in factors:
      b = 2
      if factor == 1:
        continue
      result = factor
      while result in seq:
        result = factor * b 
        b += 1
      results.append(result) 
    seq.append(min(results))
    e_t = time.perf_counter()
    d_t.append(e_t - s_t)
  return seq, d_t
		
		
	
seq, d_t = ecg_seq_index(20000)
import matplotlib.pyplot as plt
plt.plot(d_t)
plt.show()
print(d_t)