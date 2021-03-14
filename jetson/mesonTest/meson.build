project('simple', 'cuda', version : '1.0.0')

cuda_comp = meson.get_compiler('cuda')
cuda = import('unstable-cuda')

cuda = dependency('cuda', version : '>=10', modules : ['cublas'])
all_deps = [cuda]

exe = executable('prog', 'prog.cu', dependencies : all_deps)
test('cudatest', exe)
