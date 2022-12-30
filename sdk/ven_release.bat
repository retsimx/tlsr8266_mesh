
@ rem ven_release <vendor_name> <target_folder>
@ rem !!! be very careful, do not copy .svn folder !!!

@if "%1"=="" goto error_end
@if "%2"=="" goto error_end

@:begin

@echo f | xcopy /Y /Q .\boot.link %2\boot.link
@echo f | xcopy /Y /Q .\div_mod.S %2\div_mod.S

@xcopy /e /Y /Q .\proj %2\proj\

@rem use echo f to suppress manual prompt 
@echo f | xcopy /Y /Q .\proj_lib\*.h %2\proj_lib\
@echo f | xcopy /Y /Q .\proj_lib\libTL_SDK_5320.a %2\proj_lib\libTL_SDK_5320.a
@echo f | xcopy /Y /Q .\proj_lib\libTL_SDK_5328.a %2\proj_lib\libTL_SDK_5328.a
@echo f | xcopy /Y /Q .\proj_lib\libairmouse.a %2\proj_lib\libairmouse.a

@xcopy /e /Y /Q .\vendor\%1 %2\vendor\%1\
@echo f | xcopy /e /Y /Q .\.project %2\.project
@echo f | xcopy /e /Y /Q .\.cproject %2\.cproject

@echo f | xcopy /Y /Q .\vendor\common\main.c %2\vendor\common\main.c
@echo f | xcopy /Y /Q .\vendor\common\user_config_%1.h %2\vendor\common\user_config.h
@echo f | xcopy /Y /Q .\vendor\common\default_config.h %2\vendor\common\default_config.h
@echo f | xcopy /Y /Q .\vendor\common\keycode_custom_%1.h %2\vendor\common\keycode_custom.h
@echo f | xcopy /Y /Q .\vendor\common\keyboard_cfg_%1.h %2\vendor\common\keyboard_cfg.h
@echo f | xcopy /Y /Q .\vendor\common\led_cfg_%1.h %2\vendor\common\led_cfg.h

@goto done

@:error_end
@echo "usage: ven_release <vendor_name> <target_folder>"

@:done
