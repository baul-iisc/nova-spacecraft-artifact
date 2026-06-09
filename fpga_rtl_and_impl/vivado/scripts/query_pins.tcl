link_design -part xcvu9p-flga2104-2-e
foreach bank {40 42 43} {
    puts "=== Bank $bank HP IO pins ==="
    set pins [get_package_pins -filter "BANK == $bank && IS_BONDED == 1 && IS_GENERAL_PURPOSE == 1"]
    set count 0
    foreach pin $pins {
        set func [get_property PIN_FUNC $pin]
        set gc [get_property IS_GLOBAL_CLK $pin]
        set vref [get_property IS_VREF $pin]
        if {!$vref && $count < 30} {
            puts "  $pin  GC:$gc  $func"
            incr count
        }
    }
    puts "  ... total HP IO in bank $bank: [llength $pins]"
}
