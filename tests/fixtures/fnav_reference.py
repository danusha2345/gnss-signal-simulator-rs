"""Эталон страницы 1 по порядку полей ICD; не импортирует код генератора.
Запуск из корня repo: python3 tests/fixtures/fnav_reference.py.
FEC: регистр с новым битом слева, octal 171/133; матрица интерливинга 8×61.
"""
from pathlib import Path

fields = [(6,1),(6,7),(10,683),(14,5761),(31,-12345),(21,23456),(6,-7),
          (8,107),(11,562),(11,-64),(14,64),(5,18),(10,-9),(2,2),(12,1345),
          (20,0),(1,1),(26,0)]
data = ''.join(format(value & ((1 << width)-1), f'0{width}b') for width,value in fields)
assert len(data) == 214
remainder = [int(b) for b in data] + [0]*24
polynomial = [int(b) for b in format(0x1864cfb,'025b')]
for i in range(214):
    if remainder[i]:
        for j,coefficient in enumerate(polynomial):
            remainder[i+j] ^= coefficient
bits = [int(b) for b in data] + remainder[-24:] + [0]*6
state = 0
encoded = []
for bit in bits:
    register = (bit << 6) | state
    encoded += [(register & 0o171).bit_count()%2, 1 ^ ((register & 0o133).bit_count()%2)]
    state = register >> 1
assert state == 0
matrix = [[encoded[column*8+row] for column in range(61)] for row in range(8)]
transmitted = '101101110000' + ''.join(str(b) for row in matrix for b in row)
assert len(transmitted)==500
Path(__file__).with_name('fnav_page1_symbols.txt').write_text(transmitted+'\n')
