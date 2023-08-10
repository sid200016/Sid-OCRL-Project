function map(val, ilo, ihi, flo, fhi) {
    return flo + ((fhi - flo) / (ihi - ilo)) * (val - ilo)
}

function mouse_to_mm(mouse){
    if (mouse == undefined) return 0;

    let f_mouse = float(mouse)
    return int(map(f_mouse, 0, 860, -255, 255))
}

function mm_to_mouse(mm){
    if (mm == undefined) return 0;

    let f_mm = float(mm);
    return int(map(f_mm, -255, 255, 0, 860))
}