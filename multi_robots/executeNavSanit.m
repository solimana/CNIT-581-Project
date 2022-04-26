function [done] = executeNavSanit(robA_st_nums,robB_st_nums,done) %%done needed?
% done = 0;
%%
if done ==0:
    robA_running = parfeval( @robotA_path, 1, [1,2])
    robB_running = parfeval( @robotB_path, 1, [3,4])
%     sprint('a')
%     if (fetchOutputs(robA_running) == 1 && fetchOutputs(robB_running) == 1)
%         done = 1;
%     end
%     print('a1')
    
end 

%%
end