import json
import sys

ignored = []

replaced = {
    "%0 = tail call noundef i32 @llvm.arm.ldrex.p0(ptr noundef elementtype(i32) %p) #33": "%0 = load i32, ptr %p, align 4",
    "%0 = tail call noundef i32 @llvm.arm.strex.p0(i32 noundef %value, ptr noundef elementtype(i32) %addr) #33": "store i32 %value, ptr %addr, align 4\n  %0 = load i32, ptr %addr, align 4",
    "tail call void @llvm.arm.hint": "; tail call void @llvm.arm.hint",
    "tail call void asm sideeffect alignstack \"sev\"": "; tail call void asm sideeffect alignstack \"sev\"",
    "%0 = tail call i32 asm sideeffect \"mrs ${0},CONTROL\"": "%0 = tail call i32 asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call i32 asm sideeffect \"mrs ${0},EPSR\"": "%0 = tail call i32 asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call i32 asm sideeffect \"mrs ${0},IPSR\"": "%0 = tail call i32 asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call ptr asm sideeffect \"mrs ${0},MSP\"": "%0 = tail call ptr asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call i32 asm sideeffect \"mrs ${0},PRIMASK\"": "%0 = tail call i32 asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call i32 asm sideeffect \"mrs ${0},xPSR\"": "%0 = tail call i32 asm sideeffect \"add sp, ${0}\"",
    "%0 = tail call ptr asm sideeffect \"mrs ${0},PSP\"": "%0 = tail call ptr asm sideeffect \"add sp, ${0}\"",
    "tail call void asm sideeffect \"msr CONTROL, ${0}\"": "tail call void asm sideeffect \"add sp, ${0}\"",
    "tail call void asm sideeffect \"msr MSP, ${0}\"": "tail call void asm sideeffect \"add sp, ${0}\"",
    "tail call void asm sideeffect \"msr PRIMASK, ${0}\"": "tail call void asm sideeffect \"add sp, ${0}\"",
    "tail call void asm sideeffect \"msr PSP, ${0}\"": "tail call void asm sideeffect \"add sp, ${0}\"",
}

inputfile = sys.argv[-1]
with open(inputfile + ".ll2", 'w') as outputfile:
    for line in open(inputfile, 'r'):
        for kind in ignored:
            if kind in line or kind == line:
                line = '# ' + line
                break

        for find, replace in replaced.items():
            if find in line or find == line:
                line = line.replace(find, replace)

                break

        outputfile.writelines([line])
